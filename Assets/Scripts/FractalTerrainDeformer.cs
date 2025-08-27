// Assets/Scripts/Fractal/FractalTerrainDeformer.cs
using System.Collections.Generic;
using UnityEngine;

[ExecuteAlways]
[RequireComponent(typeof(MeshFilter))]
[DisallowMultipleComponent]
public class FractalTerrainDeformer : MonoBehaviour
{
    public enum ControlSource { Constant, VertexColorAlpha, TextureRed, ExternalArray }

    [Header("Profile & Control")]
    public FractalNoiseProfile profile;
    public ControlSource controlSource = ControlSource.Constant;

    [Tooltip("Used when ControlSource=Constant, or as a fallback if sources are missing.")]
    [Range(0f, 1f)] public float constantControl = 0.5f;

    [Tooltip("When using TextureRed, sample this texture at UV0 (R channel as control).")]
    public Texture2D controlTexture;

    [Tooltip("Multiply world-space displacement (after profile amplitude).")]
    public float displaceScale = 1.0f;

    [Header("Sampling")]
    [Tooltip("Scales mesh local XZ into noise domain (acts like tiling).")]
    public float uvScale = 1.0f;

    [Tooltip("Random seed for noise offset.")]
    public int seed = 12345;

    [Header("Execution")]
    public bool applyEveryFrame = false; // true for live tweaking; false for one-time bake
    public bool writeVertexColorsForDebug = false; // writes control in vertex color A for visual checks

    // Internal
    private MeshFilter _mf;
    private Mesh _meshInstance;
    private Vector3[] _baseVertices; // original verts (object space)
    private Vector3[] _baseNormals;
    private Vector2[] _uvs;
    private Color[] _colors;         // only if present
    private float[] _externalControl; // optional externally supplied per-vertex control [0..1]
    private System.Random _rng;
    private Vector2 _seedOffset;

    // Immutable originals captured once per mesh topology
    private Vector3[] _baseVerticesOrig;
    private Vector3[] _baseNormalsOrig;
    private Vector2[] _uvsOrig;

    // Scratch buffers reused each apply (avoid GC)
    private Vector3[] _workVertices;


    void OnEnable()
    {
        _mf = GetComponent<MeshFilter>();
        EnsureMeshInstance();
        InitSeed();
        CacheBaseData();
        if (!applyEveryFrame)
            ApplyDeformation();
    }

    void OnDisable()
    {
        // keep mesh instance; no destruction to allow scene edits to persist
    }

    void Update()
    {
        if (applyEveryFrame) ApplyDeformation();
    }

    void OnValidate()
    {
        if (!Application.isPlaying)
        {
            _mf = GetComponent<MeshFilter>();
            EnsureMeshInstance();
            CacheBaseData();
        }
        InitSeed();
        if (!applyEveryFrame) ApplyDeformation();
    }

    [ContextMenu("Generate Once")]
    public void GenerateOnce()
    {
        applyEveryFrame = false;
        ApplyDeformation();
    }

    [ContextMenu("Revert To Base Mesh")]
    public void RevertToBaseMesh()
    {
        EnsureMeshInstance();
        if (_baseVertices == null) CacheBaseData();
        if (_baseVertices != null)
        {
            _meshInstance.SetVertices(_baseVertices);
            _meshInstance.RecalculateBounds();
            _meshInstance.RecalculateNormals();
        }
    }

    [ContextMenu("Randomize Seed & Generate")]
    public void RandomizeSeedAndGenerate()
    {
        seed = UnityEngine.Random.Range(int.MinValue / 2, int.MaxValue / 2);
        InitSeed();
        GenerateOnce();
    }

    public void SetExternalControl(float[] perVertex01)
    {
        _externalControl = perVertex01;
        if (!applyEveryFrame) ApplyDeformation();
    }

    private void EnsureMeshInstance()
    {
        if (_mf == null) _mf = GetComponent<MeshFilter>();
        if (_mf.sharedMesh == null) return;

        // If the shared mesh is already unique to us, keep it; else clone
        if (_meshInstance == null || _meshInstance != _mf.sharedMesh)
        {
            _meshInstance = Instantiate(_mf.sharedMesh);
            _meshInstance.name = _mf.sharedMesh.name + "_FractalInstance";
            _mf.sharedMesh = _meshInstance;
        }
    }

    private void CacheBaseData()
    {
        if (_mf == null || _mf.sharedMesh == null) return;
        var m = _mf.sharedMesh;

        // Only (re)capture if topology changed (vertex count mismatch or null)
        if (_baseVerticesOrig == null || _baseVerticesOrig.Length != m.vertexCount)
        {
            // Capture originals
            _baseVerticesOrig = m.vertices;   // object space
            _baseNormalsOrig = m.normals;
            _uvsOrig = m.uv;

            if (_baseNormalsOrig == null || _baseNormalsOrig.Length != m.vertexCount)
            {
                // Compute normals ONCE for the base; we won't recalc this array again
                m.RecalculateNormals();
                _baseNormalsOrig = m.normals;
            }

            // fresh scratch buffer
            _workVertices = new Vector3[m.vertexCount];
        }

        // Optional: keep a pointer to current colors for control-from-vertex-color mode
        _colors = (m.colors != null && m.colors.Length == m.vertexCount) ? m.colors : null;
    }


    private void InitSeed()
    {
        _rng = new System.Random(seed);
        _seedOffset = new Vector2(
            (float)_rng.NextDouble() * 10000f,
            (float)_rng.NextDouble() * 10000f
        );
    }

    private void ApplyDeformation()
    {
        if (profile == null || _meshInstance == null || _baseVerticesOrig == null || _baseNormalsOrig == null)
            return;

        int n = _baseVerticesOrig.Length;
        if (_workVertices == null || _workVertices.Length != n)
            _workVertices = new Vector3[n];

        // Start from immutable base each time
        System.Array.Copy(_baseVerticesOrig, _workVertices, n);

        var outColors = writeVertexColorsForDebug ? new Color[n] : null;

        // If you sample control from vertex colors, keep reading from _colors (mesh colors).
        // (No change here)
        // ... your control sampling & FBM loop ...
        for (int i = 0; i < n; i++)
        {
            float c = constantControl;
            switch (controlSource)
            {
                case ControlSource.VertexColorAlpha:
                    if (_colors != null && i < _colors.Length) c = _colors[i].a;
                    break;
                case ControlSource.TextureRed:
                    // (unchanged)
                    break;
                case ControlSource.ExternalArray:
                    if (_externalControl != null && i < _externalControl.Length) c = Mathf.Clamp01(_externalControl[i]);
                    break;
            }

            // sample noise
            Vector3 p = _baseVerticesOrig[i];
            float fbm = SampleFBM(new Vector2(p.x, p.z), c);
            float amp = profile.EvaluateAmplitude(c) * displaceScale;

            // displace along the ORIGINAL normal, not last frame's normals
            Vector3 nrm = _baseNormalsOrig[i];
            _workVertices[i] = p + nrm * (fbm * amp);

            if (outColors != null) outColors[i] = new Color(c, c, c, fbm);
        }

        // Commit once
        _meshInstance.SetVertices(_workVertices);
        if (outColors != null) _meshInstance.SetColors(outColors);
        _meshInstance.RecalculateBounds();
        _meshInstance.RecalculateNormals(); // fine to recalc mesh normals; we never use these as "base"
    }

    // Returns FBM in [0..1] with optional domain warp, with parameters influenced by control c
    private float SampleFBM(Vector2 posLocalXZ, float c01)
    {
        // Adjusted params per control
        float freqMul = profile.EvaluateRoughness(c01);
        float gainMul = profile.EvaluateGain(c01);

        float freq = profile.baseFrequency * freqMul;
        float amp = 1f;
        float sum = 0f;
        float norm = 0f;

        Vector2 p = (posLocalXZ * uvScale) + profile.perlinOffset + _seedOffset;

        // Optional domain warping
        if (profile.domainWarp)
        {
            float wx = Perlin01(p * profile.warpFrequency + new Vector2(17.3f, 91.7f));
            float wy = Perlin01(p * profile.warpFrequency + new Vector2(73.1f, 13.9f));
            p += new Vector2(wx, wy) * profile.warpAmount;
        }

        for (int o = 0; o < Mathf.Max(1, profile.octaves); o++)
        {
            float n = Perlin01(p * freq);
            sum += n * amp;
            norm += amp;
            amp *= Mathf.Clamp01(profile.gain * gainMul);
            freq *= Mathf.Max(0.0001f, profile.lacunarity);
        }

        if (norm < 1e-6f) return 0.5f;
        return sum / norm; // keep in [0..1]
    }

    // Unity Perlin returns [0..1] already; we keep naming for clarity
    private static float Perlin01(Vector2 uv) => Mathf.PerlinNoise(uv.x, uv.y);
}
