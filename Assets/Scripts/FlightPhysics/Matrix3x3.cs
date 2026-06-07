using UnityEngine;

namespace FlightPhysics
{
    public struct Matrix3x3
    {
        public float m00, m01, m02;
        public float m10, m11, m12;
        public float m20, m21, m22;

        public Matrix3x3(
            float m00, float m01, float m02,
            float m10, float m11, float m12,
            float m20, float m21, float m22)
        {
            this.m00 = m00; this.m01 = m01; this.m02 = m02;
            this.m10 = m10; this.m11 = m11; this.m12 = m12;
            this.m20 = m20; this.m21 = m21; this.m22 = m22;
        }

        public static Matrix3x3 Identity => new Matrix3x3(
            1, 0, 0,
            0, 1, 0,
            0, 0, 1);

        public static Matrix3x3 Zero => new Matrix3x3(
            0, 0, 0,
            0, 0, 0,
            0, 0, 0);

        public float this[int row, int col]
        {
            get
            {
                switch (row * 3 + col)
                {
                    case 0: return m00;
                    case 1: return m01;
                    case 2: return m02;
                    case 3: return m10;
                    case 4: return m11;
                    case 5: return m12;
                    case 6: return m20;
                    case 7: return m21;
                    case 8: return m22;
                    default: return 0;
                }
            }
            set
            {
                switch (row * 3 + col)
                {
                    case 0: m00 = value; break;
                    case 1: m01 = value; break;
                    case 2: m02 = value; break;
                    case 3: m10 = value; break;
                    case 4: m11 = value; break;
                    case 5: m12 = value; break;
                    case 6: m20 = value; break;
                    case 7: m21 = value; break;
                    case 8: m22 = value; break;
                }
            }
        }

        public static Matrix3x3 operator +(Matrix3x3 a, Matrix3x3 b)
        {
            return new Matrix3x3(
                a.m00 + b.m00, a.m01 + b.m01, a.m02 + b.m02,
                a.m10 + b.m10, a.m11 + b.m11, a.m12 + b.m12,
                a.m20 + b.m20, a.m21 + b.m21, a.m22 + b.m22);
        }

        public static Matrix3x3 operator -(Matrix3x3 a, Matrix3x3 b)
        {
            return new Matrix3x3(
                a.m00 - b.m00, a.m01 - b.m01, a.m02 - b.m02,
                a.m10 - b.m10, a.m11 - b.m11, a.m12 - b.m12,
                a.m20 - b.m20, a.m21 - b.m21, a.m22 - b.m22);
        }

        public static Matrix3x3 operator *(Matrix3x3 a, float s)
        {
            return new Matrix3x3(
                a.m00 * s, a.m01 * s, a.m02 * s,
                a.m10 * s, a.m11 * s, a.m12 * s,
                a.m20 * s, a.m21 * s, a.m22 * s);
        }

        public static Matrix3x3 operator *(float s, Matrix3x3 a)
        {
            return a * s;
        }

        public static Matrix3x3 operator *(Matrix3x3 a, Matrix3x3 b)
        {
            return new Matrix3x3(
                a.m00 * b.m00 + a.m01 * b.m10 + a.m02 * b.m20,
                a.m00 * b.m01 + a.m01 * b.m11 + a.m02 * b.m21,
                a.m00 * b.m02 + a.m01 * b.m12 + a.m02 * b.m22,
                a.m10 * b.m00 + a.m11 * b.m10 + a.m12 * b.m20,
                a.m10 * b.m01 + a.m11 * b.m11 + a.m12 * b.m21,
                a.m10 * b.m02 + a.m11 * b.m12 + a.m12 * b.m22,
                a.m20 * b.m00 + a.m21 * b.m10 + a.m22 * b.m20,
                a.m20 * b.m01 + a.m21 * b.m11 + a.m22 * b.m21,
                a.m20 * b.m02 + a.m21 * b.m12 + a.m22 * b.m22);
        }

        public static Vector3 operator *(Matrix3x3 m, Vector3 v)
        {
            return new Vector3(
                m.m00 * v.x + m.m01 * v.y + m.m02 * v.z,
                m.m10 * v.x + m.m11 * v.y + m.m12 * v.z,
                m.m20 * v.x + m.m21 * v.y + m.m22 * v.z);
        }

        public static Vector3 operator *(Vector3 v, Matrix3x3 m)
        {
            return new Vector3(
                v.x * m.m00 + v.y * m.m10 + v.z * m.m20,
                v.x * m.m01 + v.y * m.m11 + v.z * m.m21,
                v.x * m.m02 + v.y * m.m12 + v.z * m.m22);
        }

        public float Determinant()
        {
            return m00 * (m11 * m22 - m12 * m21)
                 - m01 * (m10 * m22 - m12 * m20)
                 + m02 * (m10 * m21 - m11 * m20);
        }

        public Matrix3x3 Transpose()
        {
            return new Matrix3x3(
                m00, m10, m20,
                m01, m11, m21,
                m02, m12, m22);
        }

        public Matrix3x3 Inverse()
        {
            float det = Determinant();
            if (Mathf.Abs(det) < 1e-10f)
                return Identity;

            float invDet = 1f / det;
            return new Matrix3x3(
                (m11 * m22 - m12 * m21) * invDet,
                (m02 * m21 - m01 * m22) * invDet,
                (m01 * m12 - m02 * m11) * invDet,
                (m12 * m20 - m10 * m22) * invDet,
                (m00 * m22 - m02 * m20) * invDet,
                (m02 * m10 - m00 * m12) * invDet,
                (m10 * m21 - m11 * m20) * invDet,
                (m01 * m20 - m00 * m21) * invDet,
                (m00 * m11 - m01 * m10) * invDet);
        }

        public static Matrix3x3 SkewSymmetric(Vector3 v)
        {
            return new Matrix3x3(
                 0,   -v.z,  v.y,
                 v.z,  0,   -v.x,
                -v.y,  v.x,  0);
        }

        public static Matrix3x3 FromQuaternion(Quaternion q)
        {
            float w = q.w, x = q.x, y = q.y, z = q.z;
            return new Matrix3x3(
                1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y),
                2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x),
                2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y));
        }

        public static Matrix3x3 Diagonal(Vector3 v)
        {
            return new Matrix3x3(
                v.x, 0, 0,
                0, v.y, 0,
                0, 0, v.z);
        }

        public override string ToString()
        {
            return $"[{m00:F4}, {m01:F4}, {m02:F4}]\n[{m10:F4}, {m11:F4}, {m12:F4}]\n[{m20:F4}, {m21:F4}, {m22:F4}]";
        }
    }
}
