using UnityEngine;
using FlightPhysics;

namespace Sensors
{
    [System.Serializable]
    public struct ImuNoiseParams
    {
        public float gyroNoiseStdDev;
        public float gyroBiasStdDev;
        public float gyroBiasWalkStdDev;
        public float accelNoiseStdDev;
        public float accelBiasStdDev;
        public float accelBiasWalkStdDev;
        public float gyroLowPassCutoffHz;
        public float accelLowPassCutoffHz;
    }

    public class SimulatedImu
    {
        private ImuNoiseParams _noise;
        private Vector3 _gyroBias;
        private Vector3 _accelBias;
        private Vector3 _filteredGyro;
        private Vector3 _filteredAccel;
        private bool _initialized;

        public Vector3 gyroscope { get; private set; }
        public Vector3 accelerometer { get; private set; }
        public Quaternion attitude { get; private set; }

        public Vector3 eulerAnglesForDisplay
        {
            get
            {
                Vector3 e = attitude.eulerAngles;
                if (e.x > 180f) e.x -= 360f;
                if (e.y > 180f) e.y -= 360f;
                if (e.z > 180f) e.z -= 360f;
                return e;
            }
        }

        public void Initialize(ImuNoiseParams noise)
        {
            _noise = noise;
            _gyroBias = new Vector3(
                RandomGaussian() * noise.gyroBiasStdDev,
                RandomGaussian() * noise.gyroBiasStdDev,
                RandomGaussian() * noise.gyroBiasStdDev);
            _accelBias = new Vector3(
                RandomGaussian() * noise.accelBiasStdDev,
                RandomGaussian() * noise.accelBiasStdDev,
                RandomGaussian() * noise.accelBiasStdDev);
            _initialized = false;
        }

        public void Update(RigidBodyState state, float dt)
        {
            Vector3 trueOmegaBody = QuaternionExt.InverseRotateVector(state.attitude, state.angularVelocity);

            _gyroBias += new Vector3(
                RandomGaussian() * _noise.gyroBiasWalkStdDev * Mathf.Sqrt(dt),
                RandomGaussian() * _noise.gyroBiasWalkStdDev * Mathf.Sqrt(dt),
                RandomGaussian() * _noise.gyroBiasWalkStdDev * Mathf.Sqrt(dt));

            Vector3 rawGyro = trueOmegaBody + _gyroBias + new Vector3(
                RandomGaussian() * _noise.gyroNoiseStdDev,
                RandomGaussian() * _noise.gyroNoiseStdDev,
                RandomGaussian() * _noise.gyroNoiseStdDev);

            if (!_initialized)
            {
                _filteredGyro = rawGyro;
                _initialized = true;
            }
            else
            {
                float alpha = 2f * Mathf.PI * _noise.gyroLowPassCutoffHz * dt /
                              (1f + 2f * Mathf.PI * _noise.gyroLowPassCutoffHz * dt);
                _filteredGyro = _filteredGyro + alpha * (rawGyro - _filteredGyro);
            }

            gyroscope = _filteredGyro;

            Vector3 gravityWorld = Vector3.down * 9.81f;
            Vector3 gravityBody = QuaternionExt.InverseRotateVector(state.attitude, gravityWorld);

            Vector3 trueAccelBody = QuaternionExt.InverseRotateVector(state.attitude,
                state.linearVelocity) / Mathf.Max(dt, 0.0001f);
            Vector3 rawAccel = gravityBody + trueAccelBody + _accelBias + new Vector3(
                RandomGaussian() * _noise.accelNoiseStdDev,
                RandomGaussian() * _noise.accelNoiseStdDev,
                RandomGaussian() * _noise.accelNoiseStdDev);

            float alphaA = 2f * Mathf.PI * _noise.accelLowPassCutoffHz * dt /
                           (1f + 2f * Mathf.PI * _noise.accelLowPassCutoffHz * dt);
            if (_initialized)
                _filteredAccel = _filteredAccel + alphaA * (rawAccel - _filteredAccel);
            else
                _filteredAccel = rawAccel;

            accelerometer = _filteredAccel;

            attitude = state.attitude;
        }

        private static float RandomGaussian()
        {
            float u1 = Random.value;
            float u2 = Random.value;
            while (u1 < 1e-10f) u1 = Random.value;
            return Mathf.Sqrt(-2f * Mathf.Log(u1)) * Mathf.Cos(2f * Mathf.PI * u2);
        }
    }
}
