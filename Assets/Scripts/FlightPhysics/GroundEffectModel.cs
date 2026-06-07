using UnityEngine;

namespace FlightPhysics
{
    [System.Serializable]
    public struct GroundEffectParams
    {
        [Header("Rotor Geometry")]
        public float propDiameter;

        [Header("Ground Effect Model")]
        public float effectiveHeightRatio;
        public float maxThrustMultiplier;
        public float dragMultiplier;
        public float raycastMaxDistance;

        [Header("Turbulence")]
        public float turbulenceAmplitude;
        public float turbulenceFrequency;
        public float verticalOscFreq;
        public float verticalOscAmplitude;

        public static GroundEffectParams Default => new GroundEffectParams
        {
            propDiameter = 0.254f,
            effectiveHeightRatio = 1.5f,
            maxThrustMultiplier = 1.35f,
            dragMultiplier = 0.3f,
            raycastMaxDistance = 5f,
            turbulenceAmplitude = 0.04f,
            turbulenceFrequency = 3f,
            verticalOscFreq = 8f,
            verticalOscAmplitude = 0.15f
        };
    }

    public struct GroundEffectResult
    {
        public float thrustMultiplier;
        public float groundDistance;
        public bool inGroundEffect;
        public Vector3 dragForceWorld;
        public Vector3 turbulenceForceWorld;
    }

    public class GroundEffectModel
    {
        private GroundEffectParams _params;
        private float _propRadius;
        private float _effectiveHeight;

        private GroundEffectResult[] _results = new GroundEffectResult[4];

        public GroundEffectResult[] Results => _results;

        public void Initialize(GroundEffectParams p)
        {
            _params = p;
            _propRadius = p.propDiameter * 0.5f;
            _effectiveHeight = p.propDiameter * p.effectiveHeightRatio;
        }

        public void UpdateAllMotors(
            Vector3[] motorWorldPositions,
            Quaternion attitude,
            Vector3 linearVelocityWorld,
            MotorConfig[] motorConfigs,
            float[] motorRpms,
            float maxRpm,
            float time)
        {
            Vector3 downDir = Vector3.down;

            for (int i = 0; i < 4; i++)
            {
                _results[i] = ComputeSingleMotor(
                    i,
                    motorWorldPositions[i],
                    downDir,
                    attitude,
                    linearVelocityWorld,
                    motorConfigs[i],
                    motorRpms[i],
                    maxRpm,
                    time);
            }
        }

        public GroundEffectResult ComputeSingleMotor(
            int motorIndex,
            Vector3 motorWorldPos,
            Vector3 rayDir,
            Quaternion attitude,
            Vector3 linearVelocityWorld,
            MotorConfig config,
            float currentRpm,
            float maxRpm,
            float time)
        {
            GroundEffectResult result = new GroundEffectResult();
            result.thrustMultiplier = 1f;
            result.groundDistance = float.MaxValue;
            result.inGroundEffect = false;

            Ray ray = new Ray(motorWorldPos, rayDir);

            if (!Physics.Raycast(ray, out RaycastHit hit, _params.raycastMaxDistance))
            {
                _results[motorIndex] = result;
                return result;
            }

            result.groundDistance = hit.distance;

            if (result.groundDistance > _effectiveHeight || result.groundDistance < 0.01f)
            {
                _results[motorIndex] = result;
                return result;
            }

            result.inGroundEffect = true;

            float h = result.groundDistance;
            float R = _propRadius;

            float ratio = R / (4f * h);
            float ratioSq = ratio * ratio;

            float gef;
            if (ratioSq < 0.99f)
            {
                gef = 1f / (1f - ratioSq);
            }
            else
            {
                gef = _params.maxThrustMultiplier;
            }

            gef = Mathf.Min(gef, _params.maxThrustMultiplier);

            float rpmRatio = currentRpm / Mathf.Max(maxRpm, 1f);
            float rpmFactor = rpmRatio * rpmRatio;

            float proximityFactor = 1f - (h / _effectiveHeight);
            proximityFactor = proximityFactor * proximityFactor;

            float turbulence = 0f;
            if (_params.turbulenceAmplitude > 0f)
            {
                float phase = time * _params.turbulenceFrequency;
                float spatialPhase = motorIndex * 1.7f;
                turbulence = Mathf.PerlinNoise(phase + spatialPhase, spatialPhase + 0.3f) * 2f - 1f;
                turbulence *= _params.turbulenceAmplitude * proximityFactor * rpmFactor;
            }

            result.thrustMultiplier = gef + turbulence;

            Vector3 lateralVel = linearVelocityWorld;
            lateralVel.y = 0f;

            float dragMag = lateralVel.magnitude * _params.dragMultiplier * proximityFactor * rpmFactor;
            result.dragForceWorld = -lateralVel.normalized * dragMag;

            if (_params.verticalOscAmplitude > 0f && proximityFactor > 0.1f)
            {
                float verticalOsc = Mathf.Sin(time * _params.verticalOscFreq + motorIndex * Mathf.PI * 0.5f);
                verticalOsc *= _params.verticalOscAmplitude * proximityFactor * rpmFactor;
                result.turbulenceForceWorld = Vector3.up * verticalOsc;
            }
            else
            {
                result.turbulenceForceWorld = Vector3.zero;
            }

            _results[motorIndex] = result;
            return result;
        }

        public float GetAverageThrustMultiplier()
        {
            float sum = 0f;
            int count = 0;
            for (int i = 0; i < 4; i++)
            {
                sum += _results[i].thrustMultiplier;
                count++;
            }
            return count > 0 ? sum / count : 1f;
        }

        public bool AnyInGroundEffect()
        {
            for (int i = 0; i < 4; i++)
            {
                if (_results[i].inGroundEffect) return true;
            }
            return false;
        }
    }
}
