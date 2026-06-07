using UnityEngine;

namespace FlightController
{
    [System.Serializable]
    public struct PidGains
    {
        public float kp;
        public float ki;
        public float kd;

        public PidGains(float p, float i, float d)
        {
            kp = p; ki = i; kd = d;
        }
    }

    public class PidController
    {
        public PidGains gains;
        public float outputMin;
        public float outputMax;
        public float integralMin;
        public float integralMax;
        public float derivativeFilterCoeff;

        private float _integral;
        private float _prevError;
        private float _prevDerivative;
        private bool _firstRun;

        public void Reset()
        {
            _integral = 0f;
            _prevError = 0f;
            _prevDerivative = 0f;
            _firstRun = true;
        }

        public float Update(float setpoint, float measurement, float dt)
        {
            return UpdateFromError(setpoint - measurement, dt);
        }

        public float UpdateFromError(float error, float dt)
        {
            if (dt <= 0f) return 0f;

            float proportional = gains.kp * error;

            _integral += error * dt;

            float rawIntegralTerm = gains.ki * _integral;

            if (rawIntegralTerm > integralMax)
            {
                _integral = integralMax / gains.ki;
                rawIntegralTerm = integralMax;
            }
            else if (rawIntegralTerm < integralMin)
            {
                _integral = integralMin / gains.ki;
                rawIntegralTerm = integralMin;
            }

            float derivative;
            if (_firstRun)
            {
                derivative = 0f;
                _firstRun = false;
            }
            else
            {
                float rawDerivative = (error - _prevError) / dt;
                float alpha = derivativeFilterCoeff * dt / (1f + derivativeFilterCoeff * dt);
                _prevDerivative = _prevDerivative + alpha * (rawDerivative - _prevDerivative);
                derivative = gains.kd * _prevDerivative;
            }

            _prevError = error;

            float output = proportional + rawIntegralTerm + derivative;

            if (output > outputMax)
            {
                float excess = output - outputMax;
                if (gains.ki > 0f && rawIntegralTerm > 0f)
                {
                    _integral -= excess / gains.ki;
                }
                output = outputMax;
            }
            else if (output < outputMin)
            {
                float deficit = outputMin - output;
                if (gains.ki > 0f && rawIntegralTerm < 0f)
                {
                    _integral -= deficit / gains.ki;
                }
                output = outputMin;
            }

            return output;
        }

        public float GetIntegral() => _integral;
    }

    [System.Serializable]
    public struct AttitudeControllerParams
    {
        [Header("Outer Loop - Quaternion Attitude")]
        public PidGains rollAttitudeGains;
        public PidGains pitchAttitudeGains;
        public PidGains yawAttitudeGains;
        public float maxRollAngle;
        public float maxPitchAngle;
        public float maxYawRate;
        public float attitudeOutputMax;

        [Header("Inner Loop - Rate")]
        public PidGains rollRateGains;
        public PidGains pitchRateGains;
        public PidGains yawRateGains;
        public float rateOutputMin;
        public float rateOutputMax;

        [Header("Anti-Windup")]
        public float integralLimitAttitude;
        public float integralLimitRate;

        [Header("Derivative Filter")]
        public float derivativeFilterCutoffHz;
    }

    public class CascadedPidFlightController
    {
        private PidController _rollAttitudePid;
        private PidController _pitchAttitudePid;
        private PidController _yawAttitudePid;

        private PidController _rollRatePid;
        private PidController _pitchRatePid;
        private PidController _yawRatePid;

        private AttitudeControllerParams _params;

        private float _desiredYawAngle;
        private bool _yawInitialized;

        public float[] motorPwmOutputs = new float[4];

        public Vector3 lastAttitudeError;
        public Vector3 lastDesiredRate;
        public Quaternion lastDesiredAttitude;

        public struct ImuData
        {
            public Vector3 gyroscope;
            public Vector3 accelerometer;
            public Quaternion attitude;
        }

        public struct StickInput
        {
            public float roll;
            public float pitch;
            public float yaw;
            public float throttle;
        }

        public void Initialize(AttitudeControllerParams p)
        {
            _params = p;
            float filterCoeff = 2f * Mathf.PI * p.derivativeFilterCutoffHz;

            _rollAttitudePid = CreatePid(p.rollAttitudeGains, -p.attitudeOutputMax, p.attitudeOutputMax,
                -p.integralLimitAttitude, p.integralLimitAttitude, filterCoeff);
            _pitchAttitudePid = CreatePid(p.pitchAttitudeGains, -p.attitudeOutputMax, p.attitudeOutputMax,
                -p.integralLimitAttitude, p.integralLimitAttitude, filterCoeff);
            _yawAttitudePid = CreatePid(p.yawAttitudeGains, -p.attitudeOutputMax, p.attitudeOutputMax,
                -p.integralLimitAttitude, p.integralLimitAttitude, filterCoeff);

            _rollRatePid = CreatePid(p.rollRateGains, p.rateOutputMin, p.rateOutputMax,
                -p.integralLimitRate, p.integralLimitRate, filterCoeff);
            _pitchRatePid = CreatePid(p.pitchRateGains, p.rateOutputMin, p.rateOutputMax,
                -p.integralLimitRate, p.integralLimitRate, filterCoeff);
            _yawRatePid = CreatePid(p.yawRateGains, p.rateOutputMin, p.rateOutputMax,
                -p.integralLimitRate, p.integralLimitRate, filterCoeff);

            _yawInitialized = false;
        }

        private PidController CreatePid(PidGains g, float outMin, float outMax,
            float intMin, float intMax, float dFilter)
        {
            return new PidController
            {
                gains = g,
                outputMin = outMin,
                outputMax = outMax,
                integralMin = intMin,
                integralMax = intMax,
                derivativeFilterCoeff = dFilter
            };
        }

        public void Reset()
        {
            _rollAttitudePid.Reset();
            _pitchAttitudePid.Reset();
            _yawAttitudePid.Reset();
            _rollRatePid.Reset();
            _pitchRatePid.Reset();
            _yawRatePid.Reset();
            _yawInitialized = false;
            for (int i = 0; i < 4; i++) motorPwmOutputs[i] = 1000f;
        }

        public void Update(StickInput input, ImuData imu, float dt)
        {
            if (!_yawInitialized)
            {
                _desiredYawAngle = ExtractYaw(imu.attitude);
                _yawInitialized = true;
            }

            _desiredYawAngle += input.yaw * _params.maxYawRate * dt;

            float desiredRollAngle = input.roll * _params.maxRollAngle;
            float desiredPitchAngle = input.pitch * _params.maxPitchAngle;

            Quaternion q_desired = BuildDesiredAttitude(desiredRollAngle, desiredPitchAngle, _desiredYawAngle);
            lastDesiredAttitude = q_desired;

            Quaternion q_err = Quaternion.Inverse(imu.attitude) * q_desired;

            if (q_err.w < 0f)
            {
                q_err = new Quaternion(-q_err.x, -q_err.y, -q_err.z, -q_err.w);
            }

            Vector3 angleError = 2f * new Vector3(q_err.x, q_err.y, q_err.z);
            lastAttitudeError = angleError;

            float desiredRollRate = _rollAttitudePid.UpdateFromError(angleError.x, dt);
            float desiredPitchRate = _pitchAttitudePid.UpdateFromError(angleError.y, dt);
            float desiredYawRate = _yawAttitudePid.UpdateFromError(angleError.z, dt);

            lastDesiredRate = new Vector3(desiredRollRate, desiredPitchRate, desiredYawRate);

            float rollRateCmd = _rollRatePid.Update(desiredRollRate, imu.gyroscope.x, dt);
            float pitchRateCmd = _pitchRatePid.Update(desiredPitchRate, imu.gyroscope.y, dt);
            float yawRateCmd = _yawRatePid.Update(desiredYawRate, imu.gyroscope.z, dt);

            MixToMotors(rollRateCmd, pitchRateCmd, yawRateCmd, input.throttle);
        }

        private Quaternion BuildDesiredAttitude(float rollDeg, float pitchDeg, float yawDeg)
        {
            Quaternion q_yaw = Quaternion.AngleAxis(yawDeg, Vector3.up);
            Quaternion q_pitch = Quaternion.AngleAxis(pitchDeg, Vector3.right);
            Quaternion q_roll = Quaternion.AngleAxis(rollDeg, Vector3.forward);

            return q_yaw * q_pitch * q_roll;
        }

        private float ExtractYaw(Quaternion q)
        {
            float sinr_cosp = 2f * (q.w * q.z + q.x * q.y);
            float cosr_cosp = 1f - 2f * (q.y * q.y + q.z * q.z);
            return Mathf.Atan2(sinr_cosp, cosr_cosp) * Mathf.Rad2Deg;
        }

        private void MixToMotors(float rollCmd, float pitchCmd, float yawCmd, float throttle)
        {
            float baseThrottle = throttle * 500f;

            float m0 = baseThrottle + rollCmd + pitchCmd - yawCmd;
            float m1 = baseThrottle - rollCmd + pitchCmd + yawCmd;
            float m2 = baseThrottle - rollCmd - pitchCmd - yawCmd;
            float m3 = baseThrottle + rollCmd - pitchCmd + yawCmd;

            motorPwmOutputs[0] = Mathf.Clamp(1000f + m0, 1000f, 2000f);
            motorPwmOutputs[1] = Mathf.Clamp(1000f + m1, 1000f, 2000f);
            motorPwmOutputs[2] = Mathf.Clamp(1000f + m2, 1000f, 2000f);
            motorPwmOutputs[3] = Mathf.Clamp(1000f + m3, 1000f, 2000f);
        }

        public float[] GetMotorPwmOutputs()
        {
            return motorPwmOutputs;
        }
    }
}
