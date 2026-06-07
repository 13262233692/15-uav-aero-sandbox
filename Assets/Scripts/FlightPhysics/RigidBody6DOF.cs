using UnityEngine;

namespace FlightPhysics
{
    public struct RigidBodyState
    {
        public Vector3 position;
        public Quaternion attitude;
        public Vector3 linearVelocity;
        public Vector3 angularVelocity;

        public static RigidBodyState Lerp(RigidBodyState a, RigidBodyState b, float t)
        {
            return new RigidBodyState
            {
                position = a.position + (b.position - a.position) * t,
                attitude = Quaternion.Slerp(a.attitude, b.attitude, t),
                linearVelocity = a.linearVelocity + (b.linearVelocity - a.linearVelocity) * t,
                angularVelocity = a.angularVelocity + (b.angularVelocity - a.angularVelocity) * t
            };
        }
    }

    public struct ForceMoment
    {
        public Vector3 force;
        public Vector3 moment;

        public static ForceMoment operator +(ForceMoment a, ForceMoment b)
        {
            return new ForceMoment
            {
                force = a.force + b.force,
                moment = a.moment + b.moment
            };
        }
    }

    public class RigidBody6DOF
    {
        public float mass;
        public Matrix3x3 inertiaBody;
        public Matrix3x3 inertiaBodyInv;
        public float dragCoeffLinear;
        public float dragCoeffAngular;

        public RigidBodyState state;

        public delegate ForceMoment ExternalForceCalculator(RigidBodyState currentState);

        private ExternalForceCalculator _externalForceCalc;

        public void Initialize(float mass, Vector3 inertiaDiag, float linDrag, float angDrag,
            ExternalForceCalculator forceCalc)
        {
            this.mass = mass;
            this.inertiaBody = Matrix3x3.Diagonal(inertiaDiag);
            this.inertiaBodyInv = Matrix3x3.Diagonal(new Vector3(
                1f / inertiaDiag.x, 1f / inertiaDiag.y, 1f / inertiaDiag.z));
            this.dragCoeffLinear = linDrag;
            this.dragCoeffAngular = angDrag;
            this._externalForceCalc = forceCalc;

            state = new RigidBodyState
            {
                position = Vector3.zero,
                attitude = Quaternion.identity,
                linearVelocity = Vector3.zero,
                angularVelocity = Vector3.zero
            };
        }

        public void StepRK4(float dt)
        {
            RigidBodyState k1 = ComputeDerivative(state);
            RigidBodyState s2 = IntegrateState(state, k1, dt * 0.5f);
            RigidBodyState k2 = ComputeDerivative(s2);
            RigidBodyState s3 = IntegrateState(state, k2, dt * 0.5f);
            RigidBodyState k3 = ComputeDerivative(s3);
            RigidBodyState s4 = IntegrateState(state, k3, dt);
            RigidBodyState k4 = ComputeDerivative(s4);

            state.position += (k1.position + 2f * k2.position + 2f * k3.position + k4.position) / 6f * dt;
            state.linearVelocity += (k1.linearVelocity + 2f * k2.linearVelocity + 2f * k3.linearVelocity + k4.linearVelocity) / 6f * dt;
            state.angularVelocity += (k1.angularVelocity + 2f * k2.angularVelocity + 2f * k3.angularVelocity + k4.angularVelocity) / 6f * dt;

            Quaternion qDot1 = k1.attitude;
            Quaternion qDot2 = k2.attitude;
            Quaternion qDot3 = k3.attitude;
            Quaternion qDot4 = k4.attitude;

            Quaternion qIntegrated = new Quaternion(
                state.attitude.x + (qDot1.x + 2f * qDot2.x + 2f * qDot3.x + qDot4.x) / 6f * dt,
                state.attitude.y + (qDot1.y + 2f * qDot2.y + 2f * qDot3.y + qDot4.y) / 6f * dt,
                state.attitude.z + (qDot1.z + 2f * qDot2.z + 2f * qDot3.z + qDot4.z) / 6f * dt,
                state.attitude.w + (qDot1.w + 2f * qDot2.w + 2f * qDot3.w + qDot4.w) / 6f * dt
            );
            state.attitude = QuaternionExt.Normalize(qIntegrated);
        }

        public void StepEuler(float dt)
        {
            RigidBodyState deriv = ComputeDerivative(state);

            state.position += deriv.position * dt;
            state.linearVelocity += deriv.linearVelocity * dt;
            state.angularVelocity += deriv.angularVelocity * dt;

            Quaternion qDot = deriv.attitude;
            state.attitude = QuaternionExt.Normalize(new Quaternion(
                state.attitude.x + qDot.x * dt,
                state.attitude.y + qDot.y * dt,
                state.attitude.z + qDot.z * dt,
                state.attitude.w + qDot.w * dt
            ));
        }

        private RigidBodyState ComputeDerivative(RigidBodyState s)
        {
            ForceMoment fm = _externalForceCalc(s);

            fm.force -= dragCoeffLinear * s.linearVelocity;

            Vector3 omegaBody = QuaternionExt.InverseRotateVector(s.attitude, s.angularVelocity);
            Vector3 momentBody = QuaternionExt.InverseRotateVector(s.attitude, fm.moment);

            momentBody -= dragCoeffAngular * omegaBody;

            Vector3 omegaCrossIomega = Vector3.Cross(omegaBody,
                inertiaBody * omegaBody);

            Vector3 alphaBody = inertiaBodyInv * (momentBody - omegaCrossIomega);
            Vector3 alphaWorld = QuaternionExt.RotateVector(s.attitude, alphaBody);

            Vector3 accelWorld = fm.force / mass;

            Quaternion qDot = QuaternionExt.Derivative(s.attitude, omegaBody);

            return new RigidBodyState
            {
                position = s.linearVelocity,
                linearVelocity = accelWorld,
                attitude = qDot,
                angularVelocity = alphaWorld
            };
        }

        private RigidBodyState IntegrateState(RigidBodyState s, RigidBodyState deriv, float dt)
        {
            return new RigidBodyState
            {
                position = s.position + deriv.position * dt,
                linearVelocity = s.linearVelocity + deriv.linearVelocity * dt,
                angularVelocity = s.angularVelocity + deriv.angularVelocity * dt,
                attitude = QuaternionExt.Normalize(new Quaternion(
                    s.attitude.x + deriv.attitude.x * dt,
                    s.attitude.y + deriv.attitude.y * dt,
                    s.attitude.z + deriv.attitude.z * dt,
                    s.attitude.w + deriv.attitude.w * dt))
            };
        }

        public Vector3 GetEulerAngles()
        {
            return state.attitude.eulerAngles;
        }
    }
}
