// Assets/Scripts/Fractal/FractalNoiseProfile.cs
using UnityEngine;

[CreateAssetMenu(fileName = "FractalNoiseProfile", menuName = "Globe/Fractal Noise Profile")]
public class FractalNoiseProfile : ScriptableObject
{
    [Header("FBM")]
    [Min(1)] public int octaves = 6;
    [Min(0.0001f)] public float baseFrequency = 1.0f; // cycles per unit
    [Min(0.0001f)] public float lacunarity = 2.0f;     // frequency multiplier per octave
    [Range(0f, 1f)] public float gain = 0.5f;          // amplitude multiplier per octave
    public Vector2 perlinOffset = Vector2.zero;        // xy offset into noise space

    [Header("Amplitude (meters or mesh units)")]
    public float amplitude = 1.0f;                     // max vertical scale before control mapping

    [Header("Control → Noise Mapping")]
    [Tooltip("Maps control (0..1) → amplitude multiplier (0..1+).")]
    public AnimationCurve controlToAmplitude = AnimationCurve.Linear(0, 0, 1, 1);

    [Tooltip("Maps control (0..1) → roughness (multiplies lacunarity).")]
    public AnimationCurve controlToRoughness = AnimationCurve.Linear(0, 1, 1, 1);

    [Tooltip("Maps control (0..1) → gain override (1 = keep gain).")]
    public AnimationCurve controlToGain = AnimationCurve.Linear(0, 1, 1, 1);

    [Header("Domain Warp (optional)")]
    public bool domainWarp = false;
    public float warpAmount = 0.15f;
    public float warpFrequency = 0.75f;

    public float EvaluateAmplitude(float control01) =>
        amplitude * Mathf.Max(0f, controlToAmplitude.Evaluate(Mathf.Clamp01(control01)));

    public float EvaluateRoughness(float control01) =>
        Mathf.Max(0.0001f, controlToRoughness.Evaluate(Mathf.Clamp01(control01)));

    public float EvaluateGain(float control01) =>
        Mathf.Clamp01(controlToGain.Evaluate(Mathf.Clamp01(control01)));
}