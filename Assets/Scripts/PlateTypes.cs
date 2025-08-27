// PlateTypes.cs
using System.Collections.Generic;
using UnityEngine;

namespace Globe.Tectonics
{
    public enum PlateKind : byte { Oceanic = 0, Continental = 1 }
    public enum BoundaryType : byte { None = 0, Convergent = 1, Divergent = 2, Transform = 3 }

    public sealed class PlateConfig
    {
        public int SeedCount = 12;
        public float MinDegPerSec = 0.0f;
        public float MaxDegPerSec = 1.5f;
        public float ContinentalFraction = 0.5f; // ~equal by default
        public bool BalancedGrowth = true; // favor smaller plates when choosing which frontier grows next

    }

    public sealed class PlateState
    {
        public int PlateCount;
        public Vector3[] EulerPoles;
        public float[] DegPerSec;
        public PlateKind[] PlateKinds;     // length = PlateCount

        public int[] CellToPlate;      // length = numCells
        public Vector3[] CellVel;          // per-cell tangential velocity
        public float[] Elevation;        // per-cell elevation
    }

    public struct BoundaryInfo
    {
        public int A, B;          // neighboring cells
        public float Rate;        // +convergent, -divergent
        public BoundaryType Kind;
    }

    public sealed class PlateParams
    {
        // Base levels (arbitrary units; your shader scale maps these to meters)
        public float ContinentalBase = +0.6f;
        public float OceanicBase = -0.6f;

        // Uplift/subsidence scalars (per unit relative speed)
        public float Uplift_CC = 1.2f;   // continent–continent collision
        public float Uplift_CO = 0.9f;   // continental side uplift at C–O
        public float Uplift_OO = 0.6f;   // ocean–ocean island arcs

        public float Trench_CO = 0.7f;   // oceanic side trench at C–O
        public float Trench_OO = 0.4f;   // one oceanic side trench at O–O

        public float DivergenceSubsidence = 0.25f; // mid-ocean ridges
        public float Diffusion = 0.6f;             // erosion/isostasy blur
        public float BaseRelax = 0.05f;            // relax toward base level
        public float Dt = 0.02f;
    }

    public static class PlateSolver
    {
        // Spherical Voronoi partition from random seeds
        public static void SeedAndPartition(System.Random rng, GeodesicData g, DualData dual, PlateConfig cfg, out PlateState state)
        {
            int N = dual.CellPositions.Count;
            int K = Mathf.Clamp(cfg.SeedCount, 1, Mathf.Max(1, N));

            // --- 0) Safety: ensure neighbor graph exists
            if (g.VertexNeighbors == null || g.VertexNeighbors.Count != N)
                throw new System.InvalidOperationException("GeodesicData.VertexNeighbors not built or size mismatch with dual cells.");

            // --- 1) Choose K unique seed cells
            var seeds = new List<int>(K);
            var used = new HashSet<int>();
            while (seeds.Count < K)
            {
                int s = rng.Next(0, N);
                if (used.Add(s)) seeds.Add(s);
            }

            // --- 2) Assign plate kinds (~50/50) and kinematics
            var kinds = new PlateKind[K];
            int numC = Mathf.RoundToInt(cfg.ContinentalFraction * K);
            for (int k = 0; k < K; k++) kinds[k] = (k < numC) ? PlateKind.Continental : PlateKind.Oceanic;
            for (int k = K - 1; k > 0; k--) { int j = rng.Next(k + 1); (kinds[k], kinds[j]) = (kinds[j], kinds[k]); }

            var euler = new Vector3[K];
            var wDeg = new float[K];
            for (int k = 0; k < K; k++)
            {
                euler[k] = RandomOnSphere(rng);
                wDeg[k] = Mathf.Lerp(cfg.MinDegPerSec, cfg.MaxDegPerSec, (float)rng.NextDouble());
            }

            // --- 3) Region-growing partition
            var cellToPlate = new int[N];
            for (int i = 0; i < N; i++) cellToPlate[i] = -1;

            // Each plate maintains a "frontier" of candidate cells to annex next
            var frontiers = new List<List<int>>(K);
            var inFrontier = new bool[N]; // global guard to reduce duplicates (best effort)

            for (int k = 0; k < K; k++)
            {
                frontiers.Add(new List<int>(16));
                int s = seeds[k];
                cellToPlate[s] = k; // claim the seed
                                    // seed frontier with unclaimed neighbors
                foreach (var nb in g.VertexNeighbors[s])
                {
                    if (cellToPlate[nb] == -1 && !inFrontier[nb]) { frontiers[k].Add(nb); inFrontier[nb] = true; }
                }
            }

            // For balanced growth, we’ll prefer plates with fewer cells when choosing who grows next
            var plateSizes = new int[K];
            for (int k = 0; k < K; k++) plateSizes[k] = 1;

            int claimed = K;
            int safety = N * 10; // hard cap to avoid infinite loop in pathological cases

            while (claimed < N && safety-- > 0)
            {
                // Gather candidates: plates with non-empty frontiers
                var candidates = s_tempList; candidates.Clear();
                for (int k = 0; k < K; k++) if (frontiers[k].Count > 0) candidates.Add(k);
                if (candidates.Count == 0) break; // should not happen often; disconnected leftovers may occur on weird graphs

                int chosenPlate;
                if (cfg.BalancedGrowth)
                {
                    // 70% choose one of the smallest plates; 30% random among candidates
                    if (rng.NextDouble() < 0.7)
                    {
                        int minSize = int.MaxValue;
                        s_minList.Clear();
                        foreach (var k in candidates)
                        {
                            int sz = plateSizes[k];
                            if (sz < minSize) { minSize = sz; s_minList.Clear(); s_minList.Add(k); }
                            else if (sz == minSize) s_minList.Add(k);
                        }
                        chosenPlate = s_minList[rng.Next(s_minList.Count)];
                    }
                    else
                    {
                        chosenPlate = candidates[rng.Next(candidates.Count)];
                    }
                }
                else
                {
                    chosenPlate = candidates[rng.Next(candidates.Count)];
                }

                // Take a random cell from this plate's frontier
                var frontier = frontiers[chosenPlate];
                int idx = rng.Next(frontier.Count);
                int c = frontier[idx];
                // swap-remove for O(1)
                frontier[idx] = frontier[frontier.Count - 1];
                frontier.RemoveAt(frontier.Count - 1);

                // If already claimed by someone else (race), skip
                if (cellToPlate[c] != -1) continue;

                // Claim it
                cellToPlate[c] = chosenPlate;
                plateSizes[chosenPlate]++;
                claimed++;

                // Add its unclaimed neighbors to this plate's frontier
                foreach (var nb in g.VertexNeighbors[c])
                {
                    if (cellToPlate[nb] == -1 && !inFrontier[nb]) { frontier.Add(nb); inFrontier[nb] = true; }
                }
            }

            // If any unclaimed remain (edge case), assign them to nearest plate by adjacency
            if (claimed < N)
            {
                for (int i = 0; i < N; i++)
                {
                    if (cellToPlate[i] != -1) continue;
                    // pick first neighbor's plate or random fallback
                    int p = -1;
                    foreach (var nb in g.VertexNeighbors[i]) { if (cellToPlate[nb] != -1) { p = cellToPlate[nb]; break; } }
                    if (p == -1) p = rng.Next(0, K);
                    cellToPlate[i] = p;
                }
            }

            // --- 4) Build state
            state = new PlateState
            {
                PlateCount = K,
                EulerPoles = euler,
                DegPerSec = wDeg,
                PlateKinds = kinds,
                CellToPlate = cellToPlate,
                CellVel = new Vector3[N],
                Elevation = new float[N],
            };
        }

        // temp scratch lists (avoid GC). 
        static readonly List<int> s_tempList = new List<int>(64);
        static readonly List<int> s_minList = new List<int>(64);

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

        public static void StepTopography(
            GeodesicData g,
            DualData dual,
            PlateState st,
            List<BoundaryInfo> boundaries,
            PlateParams prm)
        {
            int N = st.Elevation.Length;

            // 1) Accumulate sources/sinks from boundaries
            var source = new float[N];

            foreach (var b in boundaries)
            {
                if (b.Kind == BoundaryType.Convergent)
                {
                    int pa = st.CellToPlate[b.A];
                    int pb = st.CellToPlate[b.B];
                    var ka = st.PlateKinds[pa];
                    var kbKind = st.PlateKinds[pb]; // <-- renamed so no conflict

                    if (ka == PlateKind.Continental && kbKind == PlateKind.Continental)
                    {
                        float s = prm.Uplift_CC * b.Rate;
                        source[b.A] += 0.5f * s;
                        source[b.B] += 0.5f * s;
                    }
                    else if (ka == PlateKind.Continental && kbKind == PlateKind.Oceanic)
                    {
                        source[b.A] += prm.Uplift_CO * b.Rate;
                        source[b.B] -= prm.Trench_CO * b.Rate;
                    }
                    else if (ka == PlateKind.Oceanic && kbKind == PlateKind.Continental)
                    {
                        source[b.B] += prm.Uplift_CO * b.Rate;
                        source[b.A] -= prm.Trench_CO * b.Rate;
                    }
                    else // O–O
                    {
                        float sUp = prm.Uplift_OO * b.Rate * 0.5f;
                        float sTrch = prm.Trench_OO * b.Rate * 0.5f;
                        source[b.A] += sUp - sTrch;
                        source[b.B] += sUp - sTrch;
                    }
                }
                else if (b.Kind == BoundaryType.Divergent)
                {
                    float s = prm.DivergenceSubsidence * (-b.Rate);
                    source[b.A] -= 0.5f * s;
                    source[b.B] -= 0.5f * s;
                }
            }

            // 2) Base level relaxation (continental high, oceanic low)
            var baseLevel = new float[N];
            for (int i = 0; i < N; i++)
            {
                int p = st.CellToPlate[i];
                baseLevel[i] = (st.PlateKinds[p] == PlateKind.Continental)
                    ? prm.ContinentalBase
                    : prm.OceanicBase;
            }

            // 3) Explicit diffusion + base relax + sources
            float dt = prm.Dt;
            float kDiff = prm.Diffusion;
            float kBase = prm.BaseRelax;   // renamed var so no clash

            var next = new float[N];
            for (int i = 0; i < N; i++)
            {
                float lap = 0f;
                var nbrs = g.VertexNeighbors[i];
                for (int t = 0; t < nbrs.Count; t++)
                    lap += (st.Elevation[nbrs[t]] - st.Elevation[i]);

                float towardBase = (baseLevel[i] - st.Elevation[i]) * kBase;

                next[i] = st.Elevation[i] + (source[i] + kDiff * lap + towardBase) * dt;
            }

            // Commit
            for (int i = 0; i < N; i++) st.Elevation[i] = next[i];
        }
    }
}
