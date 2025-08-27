using System.Collections.Generic;
using UnityEngine;

namespace Globe.Tectonics
{
    [ExecuteAlways]
    [RequireComponent(typeof(GeodesicDualSphere))]
    public class PlateSystem : MonoBehaviour
    {
        [Header("Plate Setup")]
        public int seedCount = 12;
        public int randomSeed = 12345;
        public float minDegPerSec = 0.0f;
        public float maxDegPerSec = 1.2f;
        [Range(0f, 1f)] public float continentalFraction = 0.5f; // NEW

        [Header("Topography")]
        public float continentalBase = +0.6f; // NEW
        public float oceanicBase = -0.6f; // NEW
        public float uplift_CC = 1.2f;
        public float uplift_CO = 0.9f;
        public float uplift_OO = 0.6f;
        public float trench_CO = 0.7f;
        public float trench_OO = 0.4f;
        public float divergenceSubs = 0.25f;
        public float diffusion = 0.6f;
        public float baseRelax = 0.05f; // NEW
        public float dt = 0.02f;
        public bool runSimulation = true;
        public bool colorBoundaries = true;

        private GeodesicDualSphere _gen;
        private PlateState _state;
        private PlateParams _params = new PlateParams();
        private PlateConfig _cfg = new PlateConfig();
        private List<BoundaryInfo> _boundaries = new List<BoundaryInfo>();
        private System.Random _rng;

        // Track topology changes
        private int _lastCellCount = -1;

        void OnEnable()
        {
            _gen = GetComponent<GeodesicDualSphere>();
            if (_rng == null) _rng = new System.Random(randomSeed);
            EnsureReady();
        }

        void OnValidate()
        {
            if (_rng == null) _rng = new System.Random(randomSeed);
            EnsureReady();
        }

        void Update()
        {
            if (!EnsureReady()) return;

            // keep params in sync with inspector
            _params.ContinentalBase = continentalBase;
            _params.OceanicBase = oceanicBase;
            _params.Uplift_CC = uplift_CC;
            _params.Uplift_CO = uplift_CO;
            _params.Uplift_OO = uplift_OO;
            _params.Trench_CO = trench_CO;
            _params.Trench_OO = trench_OO;
            _params.DivergenceSubsidence = divergenceSubs;
            _params.Diffusion = diffusion;
            _params.BaseRelax = baseRelax;
            _params.Dt = dt;

            // velocities + boundaries + topo step
            PlateSolver.UpdateVelocities(_gen.Dual, _state);
            PlateSolver.ClassifyBoundaries(_gen.Geodesic, _gen.Dual, _state, _boundaries);
            if (runSimulation)
                PlateSolver.StepTopography(_gen.Geodesic, _gen.Dual, _state, _boundaries, _params);

            ApplyColorsToDual();
        }

        [ContextMenu("Reseed Plates")]
        public void ReseedPlates()
        {
            if (_gen == null || _gen.Dual == null || _gen.Dual.CellPositions == null) return;

            _cfg.SeedCount = Mathf.Max(1, seedCount);
            _cfg.MinDegPerSec = minDegPerSec;
            _cfg.MaxDegPerSec = Mathf.Max(minDegPerSec, maxDegPerSec);
            _cfg.ContinentalFraction = Mathf.Clamp01(continentalFraction);

            PlateSolver.SeedAndPartition(_rng, _gen.Dual, _cfg, out _state);
            _lastCellCount = _gen.Dual.CellPositions.Count;
        }

        /// <summary>
        /// Ensure topology exists and state arrays match cell count. Rebuilds if needed.
        /// Returns true when ready for a simulation step.
        /// </summary>
        private bool EnsureReady()
        {
            if (_gen == null || _gen.Dual == null || _gen.Geodesic == null) return false;
            if (_gen.Dual.CellPositions == null) return false;

            int cellCount = _gen.Dual.CellPositions.Count;
            if (cellCount <= 0) return false;

            bool needReseed =
                _state == null ||
                _state.CellToPlate == null ||
                _state.CellVel == null ||
                _state.Elevation == null ||
                _state.CellToPlate.Length != cellCount ||
                _state.CellVel.Length != cellCount ||
                _state.Elevation.Length != cellCount ||
                _lastCellCount != cellCount;

            if (needReseed)
            {
                ReseedPlates();
                // Initialize elevation to zero after topology change
                if (_state != null && _state.Elevation != null)
                {
                    for (int i = 0; i < _state.Elevation.Length; i++) _state.Elevation[i] = 0f;
                }
                for (int i = 0; i < _state.Elevation.Length; i++)
                {
                    var p = _state.CellToPlate[i];
                    _state.Elevation[i] = (_state.PlateKinds[p] == PlateKind.Continental)
                        ? continentalBase * 0.8f
                        : oceanicBase * 0.8f;
                }
            }

            return _state != null &&
                   _state.CellToPlate != null &&
                   _state.CellVel != null &&
                   _state.Elevation != null &&
                   _state.CellToPlate.Length == cellCount &&
                   _state.CellVel.Length == cellCount &&
                   _state.Elevation.Length == cellCount;
        }

        private void ApplyColorsToDual()
        {
            var dual = _gen.Dual;
            if (dual == null || dual.DualMeshVertices == null || dual.Fans == null) return;

            var colors = new Color[dual.DualMeshVertices.Count];

            // Normalize elevation -> color
            float min = float.PositiveInfinity, max = float.NegativeInfinity;
            for (int i = 0; i < _state.Elevation.Length; i++)
            {
                float h = _state.Elevation[i];
                if (h < min) min = h; if (h > max) max = h;
            }
            float range = Mathf.Max(1e-6f, max - min);

            byte[] boundaryCode = null;
            if (colorBoundaries)
            {
                boundaryCode = new byte[_state.Elevation.Length];
                foreach (var b in _boundaries)
                {
                    boundaryCode[b.A] = (byte)Mathf.Max(boundaryCode[b.A], (byte)b.Kind);
                    boundaryCode[b.B] = (byte)Mathf.Max(boundaryCode[b.B], (byte)b.Kind);
                }
            }

            for (int cell = 0; cell < dual.Fans.Count; cell++)
            {
                var fan = dual.Fans[cell];
                if (fan.CenterVertex < 0) continue;

                float t = (_state.Elevation[cell] - min) / range;
                Color baseCol = ElevationGradient(t);

                if (colorBoundaries && boundaryCode != null && boundaryCode[cell] != 0)
                {
                    var kind = (BoundaryType)boundaryCode[cell];
                    Color tint = kind switch
                    {
                        BoundaryType.Convergent => new Color(1f, 0.25f, 0.25f),
                        BoundaryType.Divergent => new Color(0.25f, 1f, 1f),
                        BoundaryType.Transform => new Color(1f, 1f, 0.25f),
                        _ => Color.white
                    };
                    baseCol = Color.Lerp(baseCol, tint, 0.35f);
                }

                colors[fan.CenterVertex] = baseCol;
                for (int k = 0; k < fan.RimCount; k++)
                    colors[fan.RimStart + k] = baseCol;
            }

            var dualGO = transform.Find("Dual (tiles)");
            if (!dualGO) return;
            var mf = dualGO.GetComponent<MeshFilter>();
            if (!mf || !mf.sharedMesh) return;
            var mesh = mf.sharedMesh;
            if (mesh.colors == null || mesh.colors.Length != colors.Length)
                mesh.colors = colors;
            else
                mesh.SetColors(colors);
        }

        private static Color ElevationGradient(float t)
        {
            t = Mathf.Clamp01(t);
            if (t < 0.33f) return Color.Lerp(new Color(0.0f, 0.1f, 0.5f), new Color(0.0f, 0.5f, 0.7f), t / 0.33f);
            if (t < 0.66f) return Color.Lerp(new Color(0.1f, 0.6f, 0.2f), new Color(0.4f, 0.3f, 0.1f), (t - 0.33f) / 0.33f);
            return Color.Lerp(new Color(0.6f, 0.5f, 0.4f), Color.white, (t - 0.66f) / 0.34f);
        }
    }
}
