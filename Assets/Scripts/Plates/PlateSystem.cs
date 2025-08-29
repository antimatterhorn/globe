using UnityEngine;

namespace Globe.Tectonics
{
    [ExecuteAlways]
    [AddComponentMenu("Globe/Plate System")]
    [DisallowMultipleComponent]
    public partial class PlateSystem : MonoBehaviour
    {
        public enum ViewMode { Continents, Stress, Noise, Combined }

        [Header("Plate Setup")]
        public int seedCount = 12;
        public int randomSeed = 12345;
        public float minDegPerSec = 0.0f;
        public float maxDegPerSec = 1.2f;
        [Range(0f, 1f)] public float continentalFraction = 0.5f;

        [Header("View")]
        public ViewMode view = ViewMode.Continents;

        [Header("Continent Colors")]
        public Color continentalColor = new Color(0.75f, 0.65f, 0.5f);
        public Color oceanicColor = new Color(0.15f, 0.35f, 0.9f);

        [Header("Stress Colors")]
        public Color stressCold = new Color(0.2f, 0.45f, 1.0f);
        public Color stressMid = new Color(0.5f, 0.5f, 0.5f);
        public Color stressHot = new Color(1.0f, 0.25f, 0.25f);
        [Range(0f, 1f)] public float stressTint = 1f;     // visual blend in Stress view
        [Tooltip("Scale factor applied to stress in the Combined view.")]
        public float stressScale = 0.5f;

        [Header("Stress (Spatial Falloff)")]
        [Min(0)] public int stressBlurIterations = 1;          // 0 = crisp edges
        [Range(0f, 1f)] public float stressClipQuantile = 0.98f;  // robust normalization

        [Header("Noise")]
        public bool enableNoiseGeneration = true;
        public int noiseSeed = 424242;
        public float noiseFreq = 2.0f;
        [Min(1)] public int noiseOctaves = 5;
        public float noiseLacunarity = 1.9f;
        public bool perPlateOffsets = true;

        [Header("Noise Tone Mapping")]
        [Tooltip("Contrast around mid-gray (1 = neutral).")]
        [Range(0f, 3f)] public float noiseContrast = 1.0f;

        [Tooltip("Gamma on [0..1] noise (1 = neutral, <1 brighter, >1 darker).")]
        [Range(0.1f, 3f)] public float noiseGamma = 1.0f;

        [Header("Noise View Colors")]
        public Color noiseLowColor = new Color(0.1f, 0.1f, 0.1f);
        public Color noiseHighColor = new Color(0.9f, 0.9f, 0.9f);
        [Tooltip("Scale factor applied to (zero-centered) noise in the Combined view.")]
        public float noiseScale = 0.5f;

        [Header("Combined View")]
        [Tooltip("Anything at/below this is drawn as flat ocean color.")]
        [Range(-1f, 1f)] public float oceanHeight = 0.45f;
        public Color oceanSurfaceColor = new Color(0.1f, 0.3f, 0.6f);
        public Color landLowColor = new Color(0.3f, 0.6f, 0.2f);
        public Color landHighColor = new Color(0.9f, 0.9f, 0.8f);
        [Tooltip("How quickly land color ramps above oceanHeight.")]
        public float landVisualScale = 1.0f;

        // --- State & caches ---
        private GeodesicDualSphere _gen;
        private PlateState _state;
        private PlateConfig _cfg = new PlateConfig();
        private System.Random _rng;

        private float[] _stress;        // [-1,+1]
        private float[] _noise01;       // [0..1]
        private Vector3[] _plateNoiseOffset;
        private int _lastCellCount = -1;

        // ---------- Unity lifecycle ----------
        void OnEnable()
        {
            _gen = GetComponent<GeodesicDualSphere>();
            if (_rng == null) _rng = new System.Random(randomSeed);
            if (!EnsureReady()) return;

            InvalidateCaches();
            ComputeNeededLayers();
            ApplyColorsToDual();
        }

        void OnValidate()
        {
            if (_rng == null) _rng = new System.Random(randomSeed);
            if (!EnsureReady()) return;

            InvalidateCaches();
            ComputeNeededLayers();
            ApplyColorsToDual();
        }

        void Update()
        {
            if (!EnsureReady()) return;

            ComputeNeededLayers();
            ApplyColorsToDual();
        }

        [ContextMenu("Reseed Plates")]
        public void ReseedPlates()
        {
            if (_gen == null || _gen.Geodesic == null || _gen.Dual == null)
            {
                Debug.LogWarning("[PlateSystem] Reseed requested but GeodesicDualSphere not ready.");
                return;
            }

            _cfg.SeedCount = Mathf.Max(1, seedCount);
            _cfg.MinDegPerSec = minDegPerSec;
            _cfg.MaxDegPerSec = Mathf.Max(minDegPerSec, maxDegPerSec);
            _cfg.ContinentalFraction = Mathf.Clamp01(continentalFraction);
            _cfg.BalancedGrowth = true;

            PlateSolver.SeedAndPartition(_rng, _gen.Geodesic, _gen.Dual, _cfg, out _state);
            _lastCellCount = _gen.Dual.CellPositions.Count;

            InvalidateCaches();
        }

        private void InvalidateCaches()
        {
            _stress = null;
            _noise01 = null;
            _plateNoiseOffset = null;
        }

        private bool EnsureReady()
        {
            if (_gen == null) return false;
            if (_gen.Dual == null || _gen.Geodesic == null) return false;
            if (_gen.Dual.CellPositions == null) return false;

            int cellCount = _gen.Dual.CellPositions.Count;
            if (cellCount <= 0) return false;

            bool needReseed =
                _state == null ||
                _state.CellToPlate == null ||
                _state.Elevation == null || // PlateState prealloc
                _state.CellToPlate.Length != cellCount ||
                _state.Elevation.Length != cellCount ||
                _lastCellCount != cellCount;

            if (needReseed) ReseedPlates();

            return _state != null &&
                   _state.CellToPlate != null &&
                   _state.Elevation != null &&
                   _state.CellToPlate.Length == cellCount &&
                   _state.Elevation.Length == cellCount;
        }

        private void ComputeNeededLayers()
        {
            if (view == ViewMode.Stress || view == ViewMode.Combined)
                ComputeStress();

            if (enableNoiseGeneration && (view == ViewMode.Noise || view == ViewMode.Combined))
                ComputeNoise();
        }
    }
}
