using UnityEngine;

[CreateAssetMenu(fileName = "WeatherParams", menuName = "Scriptable Objects/WeatherParams")]
public class WeatherParams : ScriptableObject
{
    [Header("Time & Scales")]
    [Tooltip("Simulation time multiplier. 1 = real-time; try 3600 for hours/sec, etc.")]
    public float timeScale = 3600f;

    [Tooltip("Planet radius in meters (Earth ~ 6.371e6).")]
    public float planetRadius = 6.371e6f;

    [Tooltip("Representative tropospheric thickness for energy budget (m). 8000-10000 typical.")]
    public float activeLayerHeight = 9000f;

    [Header("Rotation / Dynamics")]
    [Tooltip("Angular speed (rad/s). Earth ≈ 7.2921159e-5. Use slider to scale it.")]
    [Range(0f, 4f)]
    public float omegaScale = 1.0f;

    [Tooltip("Base angular speed (rad/s) before scaling. Set to Earth's by default.")]
    public float omegaBase = 7.2921159e-5f;

    [Tooltip("Linear wind drag (s^-1). Larger => stronger damping.")]
    public float linearDrag = 1.0f / (3.0f * 24f * 3600f); // ~ 3-day timescale

    [Tooltip("Eddy viscosity / momentum diffusion (m^2/s).")]
    public float windDiffusivity = 3.0e4f;

    [Header("Radiation & Surface")]
    [Tooltip("Top-of-atmosphere solar constant (W/m^2). Earth ~1361.")]
    public float solarConstant = 1361f;

    [Tooltip("Planetary albedo over land (0-1).")]
    [Range(0f, 1f)] public float landAlbedo = 0.28f;

    [Tooltip("Planetary albedo over ocean (0-1).")]
    [Range(0f, 1f)] public float oceanAlbedo = 0.06f;

    [Tooltip("Simple linear outgoing longwave coefficient (W/m^2/K).")]
    public float olrLinear = 2.2f;

    [Tooltip("Surface-air thermal coupling (W/m^2/K). Crude bulk exchange.")]
    public float surfaceExchange = 10f;

    [Header("Thermodynamics")]
    [Tooltip("Specific heat of air at constant pressure (J/kg/K).")]
    public float cpAir = 1004f;

    [Tooltip("Representative air density (kg/m^3).")]
    public float rhoAir = 1.2f;

    [Tooltip("Moist adiabatic lapse rate approximation (K/m). ~6.5e-3 Earth avg.")]
    public float lapseRate = 6.5e-3f;

    [Tooltip("Reference pressure for saturation (Pa), used in Clausius–Clapeyron.")]
    public float pRef = 611f; // ~ triple point water vapor pressure (Pa)

    [Tooltip("Clausius–Clapeyron constant A (unitless pre-exponential).")]
    public float satA = 2.53e11f;

    [Tooltip("Clausius–Clapeyron constant B (K).")]
    public float satB = 5.42e3f;

    [Header("Moisture")]
    [Tooltip("Bulk evaporation coefficient (kg/kg/s).")]
    public float evapCoeff = 1.0e-5f;

    [Tooltip("Condensation rate when supersaturated (s^-1).")]
    public float condenseCoeff = 1.0e-3f;

    [Tooltip("Orographic precipitation strength (kg/kg per m/s of upslope).")]
    public float orographicCoeff = 2.0e-4f;

    [Tooltip("Scalar humidity diffusion (m^2/s).")]
    public float humidityDiffusivity = 1.0e4f;

    [Header("Classification")]
    [Tooltip("Cells with elevation below this (m) are treated as ocean.")]
    public float seaLevel = 0f;

    [Header("Initialization")]
    [Tooltip("Initial near-surface air temperature (K).")]
    public float initialTempK = 288f;

    [Tooltip("Initial specific humidity (kg/kg). 0.005 ~ moderately humid.")]
    [Range(0f, 0.05f)] public float initialSpecificHumidity = 0.005f;

    [Tooltip("Small floor for pressure (Pa) if you choose to diagnose p.")]
    public float minPressure = 5.0e4f;

    // Convenience getters
    public float Omega => omegaBase * Mathf.Max(0f, omegaScale);
}
