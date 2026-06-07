using UnityEngine;

namespace FlightPhysics
{
    public enum MotorSpinDirection
    {
        CW = 1,
        CCW = -1
    }

    [System.Serializable]
    public struct MotorConfig
    {
        public Vector3 armOffset;
        public MotorSpinDirection spinDir;
        public float thrustCoefficient;
        public float torqueCoefficient;
        public float motorConstant;
        public float maxRpm;
        public float timeConstant;
    }

    public class MotorModel
    {
        public MotorConfig config;
        public float currentRpm;
        public float targetRpm;

        private float _propInertia;

        public void Initialize(MotorConfig cfg, float propInertia = 0.0001f)
        {
            config = cfg;
            currentRpm = 0f;
            targetRpm = 0f;
            _propInertia = propInertia;
        }

        public void SetTargetRpm(float rpm)
        {
            targetRpm = Mathf.Clamp(rpm, 0f, config.maxRpm);
        }

        public void SetPwm(float pwm1000to2000)
        {
            float normalized = Mathf.Clamp01((pwm1000to2000 - 1000f) / 1000f);
            targetRpm = normalized * config.maxRpm;
        }

        public void Update(float dt)
        {
            float tau = config.timeConstant;
            float alpha = dt / (tau + dt);
            currentRpm += (targetRpm - currentRpm) * alpha;
        }

        public float GetThrust()
        {
            float omega = currentRpm * 2f * Mathf.PI / 60f;
            return config.thrustCoefficient * omega * omega;
        }

        public float GetTorque()
        {
            float omega = currentRpm * 2f * Mathf.PI / 60f;
            return config.torqueCoefficient * omega * omega * (int)config.spinDir;
        }

        public Vector3 GetGyroscopicTorque(Vector3 bodyAngularVelocity)
        {
            float omega = currentRpm * 2f * Mathf.PI / 60f;
            Vector3 propSpinAxis = Vector3.up;
            Vector3 propAngularMomentum = propSpinAxis * (_propInertia * omega * (int)config.spinDir);

            return -Vector3.Cross(bodyAngularVelocity, propAngularMomentum);
        }

        public ForceMoment ComputeForceMoment(Vector3 bodyAngularVelocity)
        {
            ForceMoment fm = new ForceMoment();

            float thrust = GetThrust();
            fm.force = Vector3.up * thrust;

            float torque = GetTorque();
            fm.moment = Vector3.up * torque;

            Vector3 armCross = Vector3.Cross(config.armOffset, fm.force);
            fm.moment += armCross;

            fm.moment += GetGyroscopicTorque(bodyAngularVelocity);

            return fm;
        }
    }
}
