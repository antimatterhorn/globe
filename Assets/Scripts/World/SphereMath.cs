using UnityEngine;

namespace Globe.Tectonics
{
    public static class SphereMath
    {
        public static Vector3 NormalizeToRadius(Vector3 v, float R) => v.sqrMagnitude > 0f ? v.normalized * R : Vector3.zero;

        public static void OrthonormalBasis(Vector3 n, out Vector3 t1, out Vector3 t2)
        {
            n = n.normalized;
            t1 = Vector3.Cross(n, Vector3.up);
            if (t1.sqrMagnitude < 1e-6f) t1 = Vector3.Cross(n, Vector3.right);
            t1.Normalize();
            t2 = Vector3.Cross(n, t1).normalized;
        }

        public static float AngleOnTangent(Vector3 vOnSphere, Vector3 n, Vector3 t1, Vector3 t2)
        {
            // project to tangent plane at n and get polar angle in [−π, π]
            var p = (vOnSphere - Vector3.Dot(vOnSphere, n) * n).normalized;
            return Mathf.Atan2(Vector3.Dot(p, t2), Vector3.Dot(p, t1));
        }

        public static long EdgeKey(int i, int j)
        {
            int a = i < j ? i : j;
            int b = i < j ? j : i;
            return ((long)a << 32) | (uint)b;
        }
    }
}