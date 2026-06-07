using UnityEngine;
using FlightPhysics;
using FlightController;
using Sensors;
using DroneInput;

[RequireComponent(typeof(Rigidbody))]
public class QuadcopterSimulator : MonoBehaviour
{
    [Header("Drone Parameters")]
    public QuadcopterParams droneParams = new QuadcopterParams();

    [Header("PID Controller Parameters")]
    public AttitudeControllerParams pidParams = new AttitudeControllerParams();

    [Header("IMU Noise Parameters")]
    public ImuNoiseParams imuNoiseParams = new ImuNoiseParams();

    [Header("Simulation")]
    public bool useRK4 = true;
    public int physicsSubsteps = 4;
    public bool armOnStart = true;

    [Header("Debug Visualization")]
    public bool showDebugGizmos = true;
    public bool logTelemetry = false;
    public float telemetryLogInterval = 0.5f;

    private RigidBody6DOF _rigidBody;
    private MotorModel[] _motors = new MotorModel[4];
    private CascadedPidFlightController _flightController;
    private SimulatedImu _imu;
    private DroneInputMapper _inputMapper;
    private GroundEffectModel _groundEffect;

    private MotorConfig[] _motorConfigs;

    private bool _armed = false;
    private float _telemetryTimer;

    public Vector3 CurrentPosition => _rigidBody.state.position;
    public Quaternion CurrentAttitude => _rigidBody.state.attitude;
    public Vector3 CurrentVelocity => _rigidBody.state.linearVelocity;
    public Vector3 CurrentAngularVelocity => _rigidBody.state.angularVelocity;
    public float[] MotorRpms => new float[]
    {
        _motors[0].currentRpm, _motors[1].currentRpm,
        _motors[2].currentRpm, _motors[3].currentRpm
    };
    public float[] MotorPwms => _flightController.GetMotorPwmOutputs();
    public bool IsArmed => _armed;
    public GroundEffectResult[] GroundEffectResults => _groundEffect.Results;
    public bool InGroundEffect => _groundEffect != null && _groundEffect.AnyInGroundEffect();

    private void Awake()
    {
        InitializeSystems();
    }

    private void Start()
    {
        if (armOnStart) Arm();
    }

    private void InitializeSystems()
    {
        _rigidBody = new RigidBody6DOF();
        _rigidBody.Initialize(
            droneParams.mass,
            droneParams.inertiaDiagonal,
            droneParams.linearDrag,
            droneParams.angularDrag,
            ComputeTotalForceMoment);

        _rigidBody.state.position = transform.position;
        _rigidBody.state.attitude = transform.rotation;

        _motorConfigs = droneParams.GetMotorConfigs();
        for (int i = 0; i < 4; i++)
        {
            _motors[i] = new MotorModel();
            _motors[i].Initialize(_motorConfigs[i], droneParams.propInertia);
        }

        _flightController = new CascadedPidFlightController();
        _flightController.Initialize(pidParams);

        _imu = new SimulatedImu();
        _imu.Initialize(imuNoiseParams);

        _inputMapper = new DroneInputMapper();

        _groundEffect = new GroundEffectModel();
        _groundEffect.Initialize(droneParams.groundEffect);
    }

    private void Arm()
    {
        _armed = true;
        _flightController.Reset();
        _inputMapper.Reset();
    }

    private void Disarm()
    {
        _armed = false;
        for (int i = 0; i < 4; i++)
            _motors[i].SetTargetRpm(0f);
    }

    private void FixedUpdate()
    {
        float fixedDt = Time.fixedDeltaTime;
        float substepDt = fixedDt / physicsSubsteps;

        DroneStickInput stickInput = _inputMapper.ReadInput(fixedDt);

        if (!_armed)
        {
            if (stickInput.throttle > 0.1f)
                Arm();
            else
                return;
        }

        if (UnityEngine.Input.GetKeyDown(KeyCode.Escape))
        {
            Disarm();
            return;
        }

        _imu.Update(_rigidBody.state, fixedDt);

        CascadedPidFlightController.ImuData imuData = new CascadedPidFlightController.ImuData
        {
            gyroscope = _imu.gyroscope,
            accelerometer = _imu.accelerometer,
            attitude = _imu.attitude
        };

        _flightController.Update(
            new CascadedPidFlightController.StickInput
            {
                roll = stickInput.roll,
                pitch = stickInput.pitch,
                yaw = stickInput.yaw,
                throttle = stickInput.throttle
            },
            imuData,
            fixedDt);

        float[] pwmOutputs = _flightController.GetMotorPwmOutputs();
        for (int i = 0; i < 4; i++)
        {
            _motors[i].SetPwm(pwmOutputs[i]);
        }

        UpdateGroundEffect();

        for (int s = 0; s < physicsSubsteps; s++)
        {
            for (int i = 0; i < 4; i++)
            {
                _motors[i].Update(substepDt);
            }

            if (useRK4)
                _rigidBody.StepRK4(substepDt);
            else
                _rigidBody.StepEuler(substepDt);
        }

        transform.position = _rigidBody.state.position;
        transform.rotation = _rigidBody.state.attitude;

        if (logTelemetry)
        {
            _telemetryTimer += fixedDt;
            if (_telemetryTimer >= telemetryLogInterval)
            {
                _telemetryTimer = 0f;
                LogTelemetry();
            }
        }
    }

    private void UpdateGroundEffect()
    {
        Vector3[] motorWorldPos = new Vector3[4];
        float[] rpms = new float[4];

        for (int i = 0; i < 4; i++)
        {
            motorWorldPos[i] = _rigidBody.state.position +
                QuaternionExt.RotateVector(_rigidBody.state.attitude, _motorConfigs[i].armOffset);
            rpms[i] = _motors[i].currentRpm;
        }

        _groundEffect.UpdateAllMotors(
            motorWorldPos,
            _rigidBody.state.attitude,
            _rigidBody.state.linearVelocity,
            _motorConfigs,
            rpms,
            droneParams.maxRpm,
            Time.time);
    }

    private ForceMoment ComputeTotalForceMoment(RigidBodyState state)
    {
        ForceMoment total = new ForceMoment();

        Vector3 gravityForce = Vector3.down * droneParams.mass * droneParams.gravity;

        total.force += gravityForce;

        Vector3 omegaBody = QuaternionExt.InverseRotateVector(state.attitude, state.angularVelocity);

        GroundEffectResult[] geResults = _groundEffect.Results;

        for (int i = 0; i < 4; i++)
        {
            ForceMoment motorFm = _motors[i].ComputeForceMoment(omegaBody);

            if (geResults[i].inGroundEffect)
            {
                motorFm.force *= geResults[i].thrustMultiplier;
            }

            ForceMoment worldFm = new ForceMoment
            {
                force = QuaternionExt.RotateVector(state.attitude, motorFm.force),
                moment = QuaternionExt.RotateVector(state.attitude, motorFm.moment)
            };

            total = total + worldFm;

            if (geResults[i].inGroundEffect)
            {
                total.force += geResults[i].dragForceWorld;
                total.force += geResults[i].turbulenceForceWorld;
            }
        }

        return total;
    }

    private void LogTelemetry()
    {
        Vector3 pos = _rigidBody.state.position;
        Vector3 vel = _rigidBody.state.linearVelocity;
        Vector3 angVel = _rigidBody.state.angularVelocity;
        Vector3 euler = _imu.eulerAnglesForDisplay;

        string geInfo = "";
        if (InGroundEffect)
        {
            geInfo = $" GE:[{_groundEffect.Results[0].thrustMultiplier:F2},{_groundEffect.Results[1].thrustMultiplier:F2},{_groundEffect.Results[2].thrustMultiplier:F2},{_groundEffect.Results[3].thrustMultiplier:F2}]";
        }

        Debug.Log(
            $"[TELEMETRY] Pos:({pos.x:F2},{pos.y:F2},{pos.z:F2}) " +
            $"Vel:({vel.x:F2},{vel.y:F2},{vel.z:F2}) " +
            $"Att:({euler.x:F1},{euler.y:F1},{euler.z:F1}) " +
            $"Rate:({angVel.x:F2},{angVel.y:F2},{angVel.z:F2}) " +
            $"RPM:[{_motors[0].currentRpm:F0},{_motors[1].currentRpm:F0},{_motors[2].currentRpm:F0},{_motors[3].currentRpm:F0}]" +
            geInfo);
    }

    private void OnDrawGizmos()
    {
        if (!showDebugGizmos || _motors == null || _motors[0] == null) return;

        MotorConfig[] configs = _motorConfigs != null ? _motorConfigs : droneParams.GetMotorConfigs();
        if (configs == null) return;

        GroundEffectResult[] geResults = _groundEffect != null ? _groundEffect.Results : null;

        for (int i = 0; i < 4; i++)
        {
            Vector3 armEnd = transform.TransformPoint(configs[i].armOffset);
            Gizmos.color = configs[i].spinDir == MotorSpinDirection.CW ? Color.red : Color.blue;
            Gizmos.DrawLine(transform.position, armEnd);

            float thrustNorm = _motors[i].currentRpm / Mathf.Max(droneParams.maxRpm, 1f);
            Gizmos.color = Color.green;
            Gizmos.DrawRay(armEnd, transform.up * thrustNorm * 0.5f);

            if (geResults != null && geResults[i].inGroundEffect)
            {
                Gizmos.color = Color.cyan;
                Gizmos.DrawRay(armEnd, Vector3.down * geResults[i].groundDistance);

                Gizmos.color = Color.yellow;
                Gizmos.DrawRay(armEnd, Vector3.up * (geResults[i].thrustMultiplier - 1f) * 0.5f);
            }
        }
    }

    private void OnGUI()
    {
        if (!_armed) return;

        GUILayout.BeginArea(new Rect(10, 10, 400, 320));
        GUILayout.Label("<b>Flight Telemetry</b>", new GUIStyle(GUI.skin.label) { richText = true });

        Vector3 euler = _imu.eulerAnglesForDisplay;
        GUILayout.Label($"Roll: {euler.x:F1}°  Pitch: {euler.y:F1}°  Yaw: {euler.z:F1}°");
        GUILayout.Label($"Alt: {CurrentPosition.y:F2}m  Spd: {CurrentVelocity.magnitude:F1}m/s");
        GUILayout.Label($"Gyro: ({_imu.gyroscope.x:F2}, {_imu.gyroscope.y:F2}, {_imu.gyroscope.z:F2}) rad/s");

        float[] rpms = MotorRpms;
        GUILayout.Label($"M1:{rpms[0]:F0} M2:{rpms[1]:F0} M3:{rpms[2]:F0} M4:{rpms[3]:F0} RPM");

        if (InGroundEffect)
        {
            GUI.color = Color.cyan;
            GroundEffectResult[] ge = _groundEffect.Results;
            GUILayout.Label($"GROUND EFFECT  Dist:[{ge[0].groundDistance:F2},{ge[1].groundDistance:F2},{ge[2].groundDistance:F2},{ge[3].groundDistance:F2}]m");
            GUILayout.Label($"GE Mult:[{ge[0].thrustMultiplier:F3},{ge[1].thrustMultiplier:F3},{ge[2].thrustMultiplier:F3},{ge[3].thrustMultiplier:F3}]");
            GUI.color = Color.white;
        }

        GUILayout.Label($"Armed: {_armed}  Mode: {(useRK4 ? "RK4" : "Euler")}  Substeps: {physicsSubsteps}");
        GUILayout.Label("[WASD] Pitch/Roll  [QE] Yaw  [Space] Throttle  [Esc] Disarm");
        GUILayout.EndArea();
    }
}
