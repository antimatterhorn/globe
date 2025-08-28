using System.Collections.Generic;
using UnityEngine;

namespace Globe.Tectonics
{
    /// <summary>
    /// Inputs needed for stress computation. Keep these at NAMESPACE scope
    /// so other files can implement them without a "PlateKinematicStress." prefix.
    /// </summary>
    public interface IPlateState
    {
        int[] CellPlateId { get; }   // per cell: owning plate index
        Vector3[] PlateOmega { get; } // per plate: angular velocity ω (rad/s) in world coords
    }

    public interface IDualMesh
    {
        Vector3[] CellCenter { get; }   // per cell position on unit sphere
        int[][] CellNeighbors { get; }  // per cell neighbor indices
    }

    /// <summary>
    /// One-shot kinematic stress from relative plate motions.
    /// Returns a per-cell scalar in [-1,+1]:
    ///   +1 = strongest convergence/uplift
    ///   -1 = strongest divergence/subsidence
    ///    ~0 = transform or interior
    /// </summary>
    public static class PlateKinematicStress
    {
        public struct StressOptions
        {
            /// <summary>How many one-ring averaging passes to bleed boundary signal into interiors.</summary>
            public int blurIterations;   // default 1

            /// <summary>Quantile for robust symmetric normalization (0..1). Typical: 0.95–0.99.</summary>
            public float clipQuantile;   // default 0.98
        }

        /// <summary>
        /// Convenience overload with defaults (blur=1, clip=0.98).
        /// </summary>
        public static float[] Compute(IDualMesh dual, IPlateState st)
        {
            var opts = new StressOptions { blurIterations = 1, clipQuantile = 0.98f };
            return Compute(dual, st, opts);
        }

        /// <summary>
        /// Main computation with options.
        /// </summary>
        public static float[] Compute(IDualMesh dual, IPlateState st, StressOptions opts)
        {
            var centers = dual.CellCenter;
            var nbrs = dual.CellNeighbors;
            var plateOf = st.CellPlateId;
            var omega = st.PlateOmega;

            int n = centers.Length;
            var raw = new float[n];

            // Per-cell accumulation of normal relative velocities across inter-plate edges
            for (int c = 0; c < n; ++c)
            {
                int pA = plateOf[c];
                if (pA < 0 || pA >= omega.Length) continue;

                Vector3 rA = centers[c].normalized;
                Vector3 vA = Vector3.Cross(omega[pA], rA);

                float acc = 0f;
                int cnt = 0;

                var neigh = nbrs[c];
                for (int k = 0; k < neigh.Length; ++k)
                {
                    int j = neigh[k];
                    if (j < 0 || j >= n) continue;

                    int pB = plateOf[j];
                    if (pB == pA || pB < 0 || pB >= omega.Length) continue; // only across plate boundaries

                    Vector3 rB = centers[j].normalized;
                    Vector3 vB = Vector3.Cross(omega[pB], rB);

                    // Boundary-normal direction in tangent plane at rA, pointing from A→B
                    Vector3 tDir = rB - Vector3.Dot(rB, rA) * rA; // project rB onto tangent at rA
                    float tLen = tDir.magnitude;
                    if (tLen < 1e-8f) continue;
                    tDir /= tLen;

                    // Relative normal approach (>0) / separation (<0)
                    float s = Vector3.Dot((vB - vA), tDir);
                    acc += s;
                    cnt++;
                }

                raw[c] = (cnt > 0) ? acc / cnt : 0f;
            }

            // Spatial falloff / bleed (optional)
            int iters = Mathf.Max(0, opts.blurIterations);
            if (iters > 0)
                raw = OneRingAverage(raw, nbrs, iters);

            // Robust symmetric normalization to [-1,+1]
            float clip = Mathf.Clamp01(opts.clipQuantile);
            return NormalizeSymmetric(raw, clip);
        }

        // -------- helpers --------

        static float[] OneRingAverage(float[] src, int[][] nbrs, int iterations)
        {
            int n = src.Length;
            var a = (float[])src.Clone();
            var b = new float[n];

            for (int it = 0; it < iterations; ++it)
            {
                for (int i = 0; i < n; ++i)
                {
                    float sum = a[i];
                    int cnt = 1;
                    var nn = nbrs[i];
                    for (int k = 0; k < nn.Length; ++k)
                    {
                        int j = nn[k];
                        if ((uint)j < (uint)n)
                        {
                            sum += a[j];
                            cnt++;
                        }
                    }
                    b[i] = sum / Mathf.Max(1, cnt);
                }
                var tmp = a; a = b; b = tmp;
            }
            return a;
        }

        static float[] NormalizeSymmetric(float[] x, float clipQuantile)
        {
            int n = x.Length;
            if (n == 0) return x;

            // collect magnitudes
            var mags = new List<float>(n);
            for (int i = 0; i < n; i++) mags.Add(Mathf.Abs(x[i]));
            mags.Sort();

            int idx = Mathf.Clamp((int)(clipQuantile * (n - 1)), 0, n - 1);
            float denom = Mathf.Max(1e-6f, mags[idx]);

            var y = new float[n];
            for (int i = 0; i < n; i++)
            {
                float v = x[i] / denom;
                y[i] = Mathf.Clamp(v, -1f, 1f);
            }
            return y;
        }
    }
}
