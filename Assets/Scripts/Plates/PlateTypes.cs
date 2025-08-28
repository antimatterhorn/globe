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

        public Vector3[] PlateNoiseOffset;   // length = PlateCount, random 3D offsets
    }

    public sealed class PlateTerrainParams
    {
        public float ContinentalAmp = 0.45f;   // base amplitude added inside continents
        public float OceanicAmp = 0.18f;   // base amplitude inside oceans

        public float NearBoundaryBoost = 0.6f; // extra relief within a few rings of boundaries
        public int BoundaryRings = 2;    // graph distance in cell steps

        public float Freq = 2.0f;              // noise frequency in 3D unit-sphere space
        public int Octaves = 5;
        public float Lacunarity = 1.9f;
        public float Gain = 0.5f;

        public bool Use4D = true;
        public float TimeSpeed = 0.05f;        // how fast the 4th dimension moves
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

            var offsets = new Vector3[K];
            for (int k = 0; k < K; k++)
            {
                // random offset in 3D noise space so patterns differ per plate
                offsets[k] = new Vector3(
                    (float)rng.NextDouble() * 100f + 13.37f * k,
                    (float)rng.NextDouble() * 100f + 42.42f * k,
                    (float)rng.NextDouble() * 100f + 7.77f * k
                );
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
                PlateNoiseOffset = offsets,
            };
        }

        // temp scratch lists (avoid GC). 
        static readonly List<int> s_tempList = new List<int>(64);
        static readonly List<int> s_minList = new List<int>(64);

        public static void ComputeBoundaryRings(GeodesicData g, PlateState st, int maxRings, int[] outRings)
        {
            int N = st.CellToPlate.Length;
            for (int i = 0; i < N; i++) outRings[i] = int.MaxValue;

            var queue = new System.Collections.Generic.Queue<int>();

            // ring 0: cells that border a different plate
            for (int i = 0; i < N; i++)
            {
                int p = st.CellToPlate[i];
                foreach (var nb in g.VertexNeighbors[i])
                {
                    if (st.CellToPlate[nb] != p)
                    {
                        outRings[i] = 0;
                        queue.Enqueue(i);
                        break;
                    }
                }
            }

            // BFS expand up to maxRings
            while (queue.Count > 0)
            {
                int c = queue.Dequeue();
                int r = outRings[c];
                if (r >= maxRings) continue;
                foreach (var nb in g.VertexNeighbors[c])
                {
                    if (outRings[nb] > r + 1)
                    {
                        outRings[nb] = r + 1;
                        queue.Enqueue(nb);
                    }
                }
            }
        }

        // Replace FBM with this version (no Simplex dependency)
        static float FBM(Vector3 p3, float t, PlateTerrainParams prm)
        {
            // Time drift: move the sample point through 3D space
            Vector3 p = p3 + new Vector3(0.73f * t * (prm.Use4D ? prm.TimeSpeed : 0f),
                                         1.21f * t * (prm.Use4D ? prm.TimeSpeed : 0f),
                                         0.53f * t * (prm.Use4D ? prm.TimeSpeed : 0f));

            float sum = 0f, amp = 1f, freq = prm.Freq;
            for (int o = 0; o < Mathf.Max(1, prm.Octaves); o++)
            {
                float n = Fake3D(p * freq); // ~[-1,1]
                sum += n * amp;
                amp *= prm.Gain;
                freq *= prm.Lacunarity;
            }

            // normalize geometric series
            float norm = (1f - Mathf.Pow(prm.Gain, Mathf.Max(1, prm.Octaves))) / (1f - prm.Gain);
            if (norm <= 1e-6f) norm = 1f;
            return sum / norm; // ~[-1,1]
        }

        // Perlin-based "fake 3D" in [-1,1] by blending three orthogonal 2D samples
        static float Fake3D(Vector3 p)
        {
            float a = Mathf.PerlinNoise(p.x, p.y);
            float b = Mathf.PerlinNoise(p.y, p.z);
            float c = Mathf.PerlinNoise(p.z, p.x);
            return ((a + b + c) / 3f) * 2f - 1f; // to [-1,1]
        }


        public static void AddSubplateRelief(
            DualData dual,
            GeodesicData g,
            PlateState st,
            PlateTerrainParams prm,
            float timeSeconds,
            float strength,                  // how hard to blend this frame (e.g., 1.0f)
            int[] scratchRings               // temp array length = cell count
        )
        {
            int N = st.CellToPlate.Length;
            if (scratchRings == null || scratchRings.Length != N) return;

            ComputeBoundaryRings(g, st, prm.BoundaryRings, scratchRings);

            // 4th noise dimension time
            float t = prm.Use4D ? timeSeconds * prm.TimeSpeed : 0f;

            for (int i = 0; i < N; i++)
            {
                int p = st.CellToPlate[i];
                Vector3 r = dual.CellPositions[i].normalized;

                // per-plate offset to decorrelate patterns
                Vector3 offs = st.PlateNoiseOffset[p];
                float baseAmp = (st.PlateKinds[p] == PlateKind.Continental) ? prm.ContinentalAmp : prm.OceanicAmp;

                // near-boundary boost (0 at far interior, up to NearBoundaryBoost at ring 0)
                int ring = scratchRings[i];
                float near = 0f;
                if (ring <= prm.BoundaryRings)
                {
                    float u = 1f - (ring / Mathf.Max(1f, (float)prm.BoundaryRings)); // 1 at ring 0 → 0 at max
                    near = u * prm.NearBoundaryBoost;
                }

                float h = FBM(r + offs, t, prm);        // [-1,1]
                float amp = baseAmp + near;             // amplitude per cell
                float delta = h * amp * strength;       // signed

                // Add gently: treat sub-plate relief as a source term toward that “detail” field
                st.Elevation[i] += delta * 0.0f; // If you want to add directly, set multiplier; otherwise accumulate in StepTopography below
            }
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

        public static void StepTopography(
            GeodesicData g,
            DualData dual,
            PlateState st,
            List<BoundaryInfo> boundaries,
            PlateParams prm,
            PlateTerrainParams tprm,
            float timeSeconds)
        {
            int N = st.Elevation.Length;

            // 1) Boundary sources/sinks
            var source = new float[N];
            foreach (var b in boundaries)
            {
                if (b.Kind == BoundaryType.Convergent)
                {
                    int pa = st.CellToPlate[b.A];
                    int pb = st.CellToPlate[b.B];
                    var ka = st.PlateKinds[pa];
                    var kbKind = st.PlateKinds[pb];

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

            // --- Sub-plate relief as an additional source term ---
            if (_ringsScratch == null || _ringsScratch.Length != N) _ringsScratch = new int[N];
            ComputeBoundaryRings(g, st, tprm.BoundaryRings, _ringsScratch);

            for (int i = 0; i < N; i++)
            {
                int p = st.CellToPlate[i];
                Vector3 r = dual.CellPositions[i].normalized;
                Vector3 offs = st.PlateNoiseOffset[p];

                float t = tprm.Use4D ? timeSeconds * tprm.TimeSpeed : 0f;
                float h = FBM(r + offs, t, tprm); // ~[-1,1]

                float baseAmp = (st.PlateKinds[p] == PlateKind.Continental) ? tprm.ContinentalAmp : tprm.OceanicAmp;

                float near = 0f;
                int ring = _ringsScratch[i];
                if (ring <= tprm.BoundaryRings)
                {
                    float u = 1f - (ring / Mathf.Max(1f, (float)tprm.BoundaryRings));
                    near = u * tprm.NearBoundaryBoost;
                }

                float amp = baseAmp + near;
                source[i] += h * amp; // inject into source
            }

            // 2) Base level relaxation
            var baseLevel = new float[N];
            for (int i = 0; i < N; i++)
            {
                int p = st.CellToPlate[i];
                baseLevel[i] = (st.PlateKinds[p] == PlateKind.Continental) ? prm.ContinentalBase : prm.OceanicBase;
            }

            // 3) Diffusion + base relax + sources
            float dt = prm.Dt;
            float kDiff = prm.Diffusion;
            float kBase = prm.BaseRelax;

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

            for (int i = 0; i < N; i++) st.Elevation[i] = next[i];
        }
        static int[] _ringsScratch;
    }

}
