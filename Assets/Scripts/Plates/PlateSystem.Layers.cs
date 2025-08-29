using System.Linq; // ToArray()
using UnityEngine;

namespace Globe.Tectonics
{
    public partial class PlateSystem
    {
        // ----- Adapters for PlateKinematicStress -----
        private sealed class DualAdaptor : IDualMesh
        {
            public Vector3[] CellCenter { get; }
            public int[][] CellNeighbors { get; }

            public DualAdaptor(GeodesicData g, DualData d)
            {
                int n = d.CellPositions.Count;
                CellCenter = d.CellPositions.ToArray();

                CellNeighbors = new int[n][];
                for (int i = 0; i < n; i++)
                {
                    var nb = g.VertexNeighbors[i];
                    var arr = new int[nb.Count];
                    for (int k = 0; k < nb.Count; k++) arr[k] = nb[k];
                    CellNeighbors[i] = arr;
                }
            }
        }

        private sealed class StateAdaptor : IPlateState
        {
            public int[] CellPlateId { get; }
            public Vector3[] PlateOmega { get; }

            public StateAdaptor(PlateState st)
            {
                CellPlateId = st.CellToPlate;
                PlateOmega = new Vector3[st.PlateCount];
                for (int p = 0; p < st.PlateCount; p++)
                {
                    Vector3 k = st.EulerPoles[p].normalized;
                    float w = st.DegPerSec[p] * Mathf.Deg2Rad;
                    PlateOmega[p] = k * w;
                }
            }
        }

        // ----- Stress -----
        private void ComputeStress()
        {
            if (_state == null || _gen == null || _gen.Dual == null || _gen.Geodesic == null) return;
            int n = _gen.Dual.CellPositions.Count;
            if (n <= 0) return;
            if (_stress != null && _stress.Length == n) return;

            var dualAd = new DualAdaptor(_gen.Geodesic, _gen.Dual);
            var stAd = new StateAdaptor(_state);

            var opts = new PlateKinematicStress.StressOptions
            {
                blurIterations = Mathf.Max(0, stressBlurIterations),
                clipQuantile = Mathf.Clamp01(stressClipQuantile)
            };
            _stress = PlateKinematicStress.Compute(dualAd, stAd, opts); // [-1,+1]
        }

        // ----- Noise -----
        // Keep this if other code still calls ComputeNoiseIfNeeded()
        private void ComputeNoiseIfNeeded() => ComputeNoise();

        // ----- Noise -----
        private void ComputeNoise()
        {
            if (!enableNoiseGeneration) return;
            if (_gen == null || _gen.Dual == null || _state == null) return;

            int n = _gen.Dual.CellPositions.Count;
            if (n <= 0) return;

            // If you want to force a rebuild whenever params change, keep this guard.
            // It’s fine to leave; just make sure params toggling invalidates caches.
            if (_noise01 != null && _noise01.Length == n && _plateNoiseOffset != null) return;

            // --- Offsets (keep or swap for the StableSeed version I posted earlier) ---
            if (perPlateOffsets)
            {
                var rng = new System.Random(noiseSeed);
                _plateNoiseOffset = new Vector3[_state.PlateCount];
                for (int p = 0; p < _state.PlateCount; p++)
                {
                    _plateNoiseOffset[p] = new Vector3(
                        (float)rng.NextDouble() * 100f + 13.37f * p,
                        (float)rng.NextDouble() * 100f + 42.42f * p,
                        (float)rng.NextDouble() * 100f + 7.77f * p
                    );
                }
            }
            else
            {
                var rng = new System.Random(noiseSeed);
                var offs = new Vector3(
                    (float)rng.NextDouble() * 100f + 11.11f,
                    (float)rng.NextDouble() * 100f + 22.22f,
                    (float)rng.NextDouble() * 100f + 33.33f
                );
                _plateNoiseOffset = new Vector3[] { offs };
            }

            _noise01 = new float[n];

            // --- Build raw fBm ---
            var raw = new float[n];
            float minv = float.PositiveInfinity, maxv = float.NegativeInfinity;

            for (int i = 0; i < n; i++)
            {
                int plate = _state.CellToPlate[i];
                Vector3 r = _gen.Dual.CellPositions[i].normalized;
                Vector3 offs = perPlateOffsets ? _plateNoiseOffset[plate] : _plateNoiseOffset[0];

                float h = FBM_NoTime(r + offs, noiseFreq, noiseOctaves, noiseLacunarity); // ~[-1,1]
                raw[i] = h;
                if (h < minv) minv = h;
                if (h > maxv) maxv = h;
            }

            // --- Normalize to [0..1] FIRST ---
            float range = maxv - minv;
            if (range < 1e-6f)
            {
                // Degenerate case: all samples equal → just set mid-gray
                for (int i = 0; i < n; i++) _noise01[i] = 0.5f;
            }
            else
            {
                for (int i = 0; i < n; i++)
                    _noise01[i] = Mathf.Clamp01((raw[i] - minv) / range);
            }

            // --- Then tone-map: gamma on 0..1, then contrast around 0.5 ---
            float g = Mathf.Max(0.1f, noiseGamma);
            float c = Mathf.Max(0f, noiseContrast);

            for (int i = 0; i < n; i++)
            {
                float u = _noise01[i];           // 0..1
                u = Mathf.Pow(u, g);             // gamma
                u = 0.5f + (u - 0.5f) * c;       // contrast
                _noise01[i] = Mathf.Clamp01(u);
            }
        }

        // fBm (static, no time) with fixed octave falloff g=0.5, normalized to ~[-1,1]
        private static float FBM_NoTime(Vector3 p, float freq, int octaves, float lacunarity)
        {
            float sum = 0f, amp = 1f, f = Mathf.Max(1e-6f, freq);
            int O = Mathf.Max(1, octaves);
            const float g = 0.5f;                       // fixed spectral falloff
            float lac = Mathf.Max(1.0001f, lacunarity);

            for (int o = 0; o < O; o++)
            {
                float n = Fake3DPerlin(p * f);          // [-1,1]
                sum += n * amp;
                amp *= g;
                f *= lac;
            }

            // geometric-series normalization for g=0.5
            float norm = (1f - Mathf.Pow(g, O)) / (1f - g);
            if (norm < 1e-6f) norm = 1f;
            return sum / norm;                           // ~[-1,1]
        }


        private static float Fake3DPerlin(Vector3 p)
        {
            float a = Mathf.PerlinNoise(p.x, p.y);
            float b = Mathf.PerlinNoise(p.y, p.z);
            float c = Mathf.PerlinNoise(p.z, p.x);
            return ((a + b + c) / 3f) * 2f - 1f; // [-1,1]
        }
    }
}
