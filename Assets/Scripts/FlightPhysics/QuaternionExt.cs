using UnityEngine;

namespace FlightPhysics
{
    public static class QuaternionExt
    {
        public static Quaternion Derivative(Quaternion q, Vector3 angularVelocityBody)
        {
            Quaternion omegaQuat = new Quaternion(angularVelocityBody.x * 0.5f,
                                                   angularVelocityBody.y * 0.5f,
                                                   angularVelocityBody.z * 0.5f,
                                                   0f);
            Quaternion result = omegaQuat * q;
            return result;
        }

        public static Quaternion Normalize(Quaternion q)
        {
            float mag = Mathf.Sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
            if (mag < 1e-10f) return Quaternion.identity;
            float invMag = 1f / mag;
            return new Quaternion(q.x * invMag, q.y * invMag, q.z * invMag, q.w * invMag);
        }

        public static Vector3 RotateVector(Quaternion q, Vector3 v)
        {
            Matrix3x3 r = Matrix3x3.FromQuaternion(q);
            return r * v;
        }

        public static Vector3 InverseRotateVector(Quaternion q, Vector3 v)
        {
            Matrix3x3 r = Matrix3x3.FromQuaternion(q).Transpose();
            return r * v;
        }
    }
}
