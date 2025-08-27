// PlateTypes.cs
using System.Collections.Generic;
using UnityEngine;

namespace Globe.Tectonics
{
    public sealed class PlateConfig
    {
        public int SeedCount = 12;
        public float MinDegPerSec = 0.0f;
        public float MaxDegPerSec = 1.5f;  // visual speed; tune freely
    }

    public sealed class PlateState
    {
        public int PlateCount;
        public Vector3[] EulerPoles;  // unit vectors
        public float[] DegPerSec;   // angular speeds
        public int[] CellToPlate; // length = number of cells
        public Vector3[] CellVel;     // per-cell tangential velocity (world units/sec)
        public float[] Elevation;   // per-cell height signal
    }

    public enum BoundaryType : byte { None = 0, Convergent = 1, Divergent = 2, Transform = 3 }

    public struct BoundaryInfo
    {
        public int A, B;           // neighboring cell indices
        public float Rate;         // + convergent, - divergent (units of speed)
        public BoundaryType Kind;
    }

    public sealed class PlateParams
    {
        public float UpliftPerUnitConvergence = 0.8f;  // height/sec per unit of convergence speed
        public float SubsidencePerUnitDivergence = 0.2f;
        public float Diffusion = 0.6f;                // simple Laplacian diffusion coefficient
        public float Dt = 0.02f;                      // seconds per simulation tick
    }

    public static class PlateSolver
    {
        // Spherical Voronoi partition from random seeds
        public static void SeedAndPartition(System.Random rng, DualData dual, PlateConfig cfg, out PlateState state)
        {
            int N = dual.CellPositions.Count;
            int K = Mathf.Clamp(cfg.SeedCount, 1, Mathf.Max(1, N));
            var seeds = new List<int>(K);
            var used = new HashSet<int>();

            while (seeds.Count < K)
            {
                int s = rng.Next(0, N);
                if (used.Add(s)) seeds.Add(s);
            }

            var cellToPlate = new int[N];
            var seedDirs = new Vector3[K];
            for (int k = 0; k < K; k++) seedDirs[k] = dual.CellPositions[seeds[k]].normalized;

            for (int i = 0; i < N; i++)
            {
                Vector3 r = dual.CellPositions[i].normalized;
                int best = 0;
                float bestDot = Vector3.Dot(r, seedDirs[0]);
                for (int k = 1; k < K; k++)
                {
                    float d = Vector3.Dot(r, seedDirs[k]); // max dot == min great-circle distance
                    if (d > bestDot) { bestDot = d; best = k; }
                }
                cellToPlate[i] = best;
            }

            // Random Euler poles and speeds
            var euler = new Vector3[K];
            var wDeg = new float[K];
            for (int k = 0; k < K; k++)
            {
                // random direction on sphere
                euler[k] = RandomOnSphere(rng);
                wDeg[k] = Mathf.Lerp(cfg.MinDegPerSec, cfg.MaxDegPerSec, (float)rng.NextDouble());
            }

            state = new PlateState
            {
                PlateCount = K,
                EulerPoles = euler,
                DegPerSec = wDeg,
                CellToPlate = cellToPlate,
                CellVel = new Vector3[N],
                Elevation = new float[N],
            };
        }

        public static Vector3 RandomOnSphere(System.Random rng)
        {
            // Marsaglia method
            float u = 2f * (float)rng.NextDouble() - 1f;
            float t = 2f * Mathf.PI * (float)rng.NextDouble();
            float s = Mathf.Sqrt(1f - u * u);
            return new Vector3(s * Mathf.Cos(t), u, s * Mathf.Sin(t));
        }

        // Compute per-cell velocity v = ω × r
        public static void UpdateVelocities(DualData dual, PlateState state)
        {
            if (dual == null || dual.CellPositions == null || state == null) return;
            int N = dual.CellPositions.Count;
            if (state.CellToPlate == null || state.CellVel == null || state.CellToPlate.Length != N || state.CellVel.Length != N)
                return; // caller should re-init PlateState for the new topology

            for (int i = 0; i < N; i++)
            {
                int p = state.CellToPlate[i];
                // Guard plate index range
                if (p < 0 || p >= state.PlateCount) { state.CellVel[i] = Vector3.zero; continue; }

                Vector3 r = dual.CellPositions[i].normalized;
                Vector3 k = state.EulerPoles[p].normalized;
                float w = state.DegPerSec[p] * Mathf.Deg2Rad; // rad/sec
                Vector3 omega = k * w;
                state.CellVel[i] = Vector3.Cross(omega, r);
            }
        }


        // Build simple neighbor list from polygon adjacency (via DualFans: neighbors are polygon rim successors)
        public static void ClassifyBoundaries(GeodesicData g, DualData dual, PlateState state, List<BoundaryInfo> outBoundaries)
        {
            outBoundaries.Clear();
            int N = dual.CellPositions.Count;
            // Use geodesic vertex neighbors (one-to-one with cells)
            for (int i = 0; i < g.VertexNeighbors.Count; i++)
            {
                foreach (var j in g.VertexNeighbors[i])
                {
                    if (j <= i) continue; // each pair once
                    int pi = state.CellToPlate[i];
                    int pj = state.CellToPlate[j];
                    if (pi == pj) continue;

                    // Midpoint on sphere
                    Vector3 ri = dual.CellPositions[i].normalized;
                    Vector3 rj = dual.CellPositions[j].normalized;
                    Vector3 rMid = (ri + rj).normalized;

                    // Boundary tangent direction (along great-circle between cells) projected to tangent plane at rMid
                    Vector3 t = (rj - ri);
                    t = (t - Vector3.Dot(t, rMid) * rMid).normalized;

                    // Normal across boundary within tangent plane (points from i->j)
                    Vector3 n = Vector3.Cross(rMid, t).normalized;

                    // Relative velocity at boundary midpoint (approx by averaging cell velocities)
                    Vector3 vi = state.CellVel[i];
                    Vector3 vj = state.CellVel[j];
                    Vector3 vRel = vj - vi;

                    float rate = Vector3.Dot(vRel, n); // +: j moves toward i (convergence), -: away (divergence)
                    BoundaryType kind;
                    const float eps = 1e-5f;
                    if (rate > eps) kind = BoundaryType.Convergent;
                    else if (rate < -eps) kind = BoundaryType.Divergent;
                    else kind = BoundaryType.Transform;

                    outBoundaries.Add(new BoundaryInfo { A = i, B = j, Rate = rate, Kind = kind });
                }
            }
        }

        public static void StepTopography(GeodesicData g, PlateState state, List<BoundaryInfo> boundaries, PlateParams prm)
        {
            int N = state.Elevation.Length;

            // Accumulate sources/sinks from boundaries
            var source = new float[N];
            foreach (var b in boundaries)
            {
                if (b.Kind == BoundaryType.Convergent)
                {
                    float s = prm.UpliftPerUnitConvergence * b.Rate;
                    source[b.A] += s * 0.5f;
                    source[b.B] += s * 0.5f;
                }
                else if (b.Kind == BoundaryType.Divergent)
                {
                    float s = prm.SubsidencePerUnitDivergence * (-b.Rate);
                    source[b.A] -= s * 0.5f;
                    source[b.B] -= s * 0.5f;
                }
                // transform: ignore or add small shear noise if desired
            }

            // Explicit diffusion (graph Laplacian on cell adjacency)
            float dt = prm.Dt;
            float k = prm.Diffusion;
            var next = new float[N];
            for (int i = 0; i < N; i++)
            {
                float lap = 0f;
                var nbrs = g.VertexNeighbors[i];
                for (int t = 0; t < nbrs.Count; t++)
                    lap += (state.Elevation[nbrs[t]] - state.Elevation[i]);
                next[i] = state.Elevation[i] + (source[i] + k * lap) * dt;
            }
            // Commit
            for (int i = 0; i < N; i++) state.Elevation[i] = next[i];
        }
    }
}
