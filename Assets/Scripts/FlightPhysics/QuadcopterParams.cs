using UnityEngine;

namespace FlightPhysics
{
    [System.Serializable]
    public class QuadcopterParams
    {
        [Header("Mass & Inertia")]
        public float mass = 1.5f;
        public Vector3 inertiaDiagonal = new Vector3(0.0232f, 0.0232f, 0.0468f);

        [Header("Aerodynamics")]
        public float linearDrag = 0.1f;
        public float angularDrag = 0.01f;

        [Header("Motor")]
        public float thrustCoeff = 1.04e-5f;
        public float torqueCoeff = 1.4e-7f;
        public float motorTimeConstant = 0.05f;
        public float maxRpm = 10000f;
        public float propInertia = 0.0001f;

        [Header("Frame Geometry (arm length in meters)")]
        public float armLength = 0.25f;

        [Header("Gravity")]
        public float gravity = 9.81f;

        public MotorConfig[] GetMotorConfigs()
        {
            float l = armLength;
            float sl = l * Mathf.Cos(Mathf.PI / 4f);
            MotorConfig[] configs = new MotorConfig[4];

            configs[0] = new MotorConfig
            {
                armOffset = new Vector3(sl, 0, sl),
                spinDir = MotorSpinDirection.CW,
                thrustCoefficient = thrustCoeff,
                torqueCoefficient = torqueCoeff,
                timeConstant = motorTimeConstant,
                maxRpm = maxRpm
            };
            configs[1] = new MotorConfig
            {
                armOffset = new Vector3(-sl, 0, sl),
                spinDir = MotorSpinDirection.CCW,
                thrustCoefficient = thrustCoeff,
                torqueCoefficient = torqueCoeff,
                timeConstant = motorTimeConstant,
                maxRpm = maxRpm
            };
            configs[2] = new MotorConfig
            {
                armOffset = new Vector3(-sl, 0, -sl),
                spinDir = MotorSpinDirection.CW,
                thrustCoefficient = thrustCoeff,
                torqueCoefficient = torqueCoeff,
                timeConstant = motorTimeConstant,
                maxRpm = maxRpm
            };
            configs[3] = new MotorConfig
            {
                armOffset = new Vector3(sl, 0, -sl),
                spinDir = MotorSpinDirection.CCW,
                thrustCoefficient = thrustCoeff,
                torqueCoefficient = torqueCoeff,
                timeConstant = motorTimeConstant,
                maxRpm = maxRpm
            };

            return configs;
        }
    }
}
