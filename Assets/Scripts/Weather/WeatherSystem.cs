using System;
using System.Collections.Generic;
using UnityEngine;
using Globe.Tectonics;

namespace Globe.Weather
{
    [RequireComponent(typeof(PlateSystem))]
    [AddComponentMenu("Globe/Weather System")]
    [DefaultExecutionOrder(100)]
    public class WeatherSystem : MonoBehaviour
    {
        [Header("Debug")]
        [SerializeField] private bool debugLogs = true;

        [Header("Parameters")]
        public WeatherParams prm;

        [SerializeField, Tooltip("Initial equator-to-pole temperature contrast (K) to break symmetry.")]
        private float initialDeltaT = 6f; // small kick: ~6K from equator to pole

        [Header("Run Control")]
        public bool runSimulation = true;
        [Tooltip("If >= 0, overrides WeatherParams.timeScale.")]
        public float timeScaleOverride = -1f;

        [Header("Diagnostics / Gizmos")]
        public bool gizmosWind = true;
        [Tooltip("If <= 0, auto-detect from dual cell positions.")]
        public float gizmoSphereRadius = -1f;
        [Tooltip("Scale for wind line length in scene units per (m/s).")]
        public float gizmoWindScale = 0.25f;
        public bool accumulateRain = true;
        public float accumulationWindowSeconds = 86400f; // 1 day

        [Header("Gizmos / Wind")]
        public bool showWindGizmos = true;
        [Min(1)] public int gizmoEveryNth = 250;        // draw 1 of every N cells
        public float gizmoLenPer100ms = 0.25f;          // world units for 100 m/s
        public float gizmoMaxLen = 0.6f;                // clamp arrow length
        public float gizmoColorMaxSpeed = 80f;          // m/s at “hot” color
        public Color gizmoWindLow = new Color(0.35f, 0.85f, 1f, 0.9f);
        public Color gizmoWindHigh = new Color(1f, 0.3f, 0.5f, 0.9f);

        // refs
        private PlateSystem plate;

        // mesh cache
        private int N;
        private Vector3[] pos;      // unit-sphere dual centers
        private Vector3[] up;       // per-cell normal
        private float[] area;       // m^2 (uniform fallback)
        private List<int>[] nbr;    // neighbor lists
        private float R;            // planet radius (m)

        // terrain
        private float[] elev;       // alias to plate.ElevationField
        private bool[] isOcean;

        // state
        private float[] T;          // K
        private float[] q;          // kg/kg
        private float[] p;          // Pa (diagnosed)
        private Vector3[] u;        // m/s (tangent)
        private float[] cloud;      // proxy
        private float[] rain;       // kg/m^2/s
        private float[] rainAccum;  // kg/m^2

        // tendencies
        private float[] dT_dt, dq_dt;
        private Vector3[] du_dt;
        private float accumTimer;

        private float[] _Tpred;

        // ---------------- Unity ----------------

        private void Reset() { plate = GetComponent<PlateSystem>(); }
        private void OnValidate() { if (!plate) plate = GetComponent<PlateSystem>(); }

        private void Awake()
        {
            if (!plate) plate = GetComponent<PlateSystem>();
            if (!prm)
            {
                Debug.LogError("[WeatherSystem] WeatherParams not assigned.");
                enabled = false;
            }
        }

        private void Start() { BootstrapIfReady(); }

        private void OnEnable()
        {
            if (!plate) plate = GetComponent<PlateSystem>();
            BootstrapIfReady();
        }


        private void Update()
        {
            if (!runSimulation) return;

            if (N == 0 || pos == null || elev == null)
            {
                BootstrapIfReady();
                if (N == 0) return;
            }

            float dt = Mathf.Max(1e-3f, Time.deltaTime) * ((timeScaleOverride >= 0f) ? timeScaleOverride : prm.timeScale);

            Array.Clear(dT_dt, 0, N);
            Array.Clear(dq_dt, 0, N);
            Array.Clear(du_dt, 0, N);
            Array.Clear(rain, 0, N);

            // 1) Heating first
            StepHeating();

            // (reuse a scratch array or allocate once at class scope)
            if (_Tpred == null || _Tpred.Length != N) _Tpred = new float[N];
            for (int i = 0; i < N; i++) _Tpred[i] = T[i] + dT_dt[i] * dt;

            // 2) Diagnose PGF using T_pred
            DiagnosePressureAndPGF(_Tpred);

            // 3) Rest as before
            StepCoriolisDragDiffusion();
            StepMoistureLocal();
            StepOrographicRain();
            AdvectScalar(q, dq_dt);
            Integrate(dt);

            SanitizeState("PGF");
            SanitizeState("Coriolis");
            SanitizeState("Integrate");

            if (debugLogs && Time.frameCount % 30 == 0)
            {
                float tmin = float.PositiveInfinity, tmax = float.NegativeInfinity;
                for (int i = 0; i < N; i++) { tmin = Mathf.Min(tmin, T[i]); tmax = Mathf.Max(tmax, T[i]); }
                Debug.Log($"[WeatherSystem] T range: {tmin:0.00}–{tmax:0.00} K, max|u|={MaxMag(u):0.###} m/s");
            }
            float MaxMag(Vector3[] arr) { float m = 0f; for (int i = 0; i < arr.Length; i++) m = Mathf.Max(m, arr[i].magnitude); return m; }

            if (accumulateRain)
            {
                accumTimer += dt;
                for (int i = 0; i < N; i++) rainAccum[i] += rain[i] * dt;
                if (accumTimer >= Mathf.Max(1f, accumulationWindowSeconds))
                {
                    Array.Clear(rainAccum, 0, N);
                    accumTimer = 0f;
                }
            }
        }

        // ---------------- Bootstrap ----------------

        private void BootstrapIfReady()
        {
            if (plate == null || !plate.IsBuilt) return;

            N = plate.CellCount;
            if (N <= 0) return;

            R = Mathf.Max(1f, prm.planetRadius);

            pos = new Vector3[N];
            up = new Vector3[N];
            for (int i = 0; i < N; i++)
            {
                pos[i] = plate.GetCellPosition(i).normalized;
                up[i] = pos[i];
            }

            elev = plate.ElevationField;
            isOcean = new bool[N];
            for (int i = 0; i < N; i++) isOcean[i] = elev[i] <= prm.seaLevel;

            // neighbors straight from Geodesic.VertexNeighbors via accessor
            nbr = new List<int>[N];
            bool anyNbr = false;
            for (int i = 0; i < N; i++)
            {
                var ni = plate.GetNeighbors(i);
                if (ni != null)
                {
                    anyNbr = true;
                    nbr[i] = new List<int>(ni);
                }
            }
            if (!anyNbr)
            {
                Debug.LogWarning("[WeatherSystem] No neighbor graph available (GetNeighbors returned null). PGF/advection/orographic will be inert.");
                nbr = new List<int>[N]; // stays null entries
            }

            // areas (uniform fallback)
            area = new float[N];
            float A = 4f * Mathf.PI * R * R;
            float Au = A / N;
            for (int i = 0; i < N; i++)
            {
                float ai = plate.GetCellArea(i);
                area[i] = (ai > 0f) ? ai : Au;
            }

            // Auto gizmo radius if user hasn't set it
            if (gizmoSphereRadius <= 0f)
                gizmoSphereRadius = AutoDetectSphereRadius();

            // Quick debug: neighbor availability & counts
            if (debugLogs)
            {
                int withNbrs = 0, nbrSum = 0;
                for (int i = 0; i < N; i++)
                {
                    var ni = nbr[i];
                    if (ni != null && ni.Count > 0) { withNbrs++; nbrSum += ni.Count; }
                }
                string nbrMsg = (withNbrs > 0) ? $"neighbors OK ({withNbrs}/{N}, avg {((float)nbrSum / Mathf.Max(1, withNbrs)):0.0})"
                                               : "NO NEIGHBORS (PGF/advection/orographic off)";
                Debug.Log($"[WeatherSystem] Bootstrapped N={N}, {nbrMsg}, gizmoRadius={gizmoSphereRadius:0.###}, R={R:0.##} m");
            }

            // state alloc/init
            T = new float[N]; q = new float[N]; p = new float[N]; u = new Vector3[N];
            cloud = new float[N]; rain = new float[N]; rainAccum = new float[N];
            dT_dt = new float[N]; dq_dt = new float[N]; du_dt = new Vector3[N];

            for (int i = 0; i < N; i++)
            {
                T[i] = prm.initialTempK;
                q[i] = prm.initialSpecificHumidity;
                p[i] = Mathf.Max(prm.minPressure, 1.0e5f);
                u[i] = Vector3.zero;
            }

            for (int i = 0; i < N; i++)
            {
                // If your spin axis is +Z, use up[i].z here (and in heating/Coriolis)
                float lat = Mathf.Asin(Mathf.Clamp(up[i].y, -1f, 1f));
                float mu = Mathf.Cos(lat); // 1 at equator, 0 at poles
                T[i] = prm.initialTempK + initialDeltaT * (mu - 0.5f); // small equator-warm bias
                q[i] = prm.initialSpecificHumidity;
                p[i] = Mathf.Max(prm.minPressure, 1.0e5f);
                u[i] = Vector3.zero;
            }
        }

        // ---------------- Physics ----------------

        private void StepHeating()
        {
            for (int i = 0; i < N; i++)
            {
                // If your spin axis is +Z, switch up[i].y to up[i].z here and in StepCoriolisDragDiffusion.
                float lat = Mathf.Asin(Mathf.Clamp(up[i].y, -1f, 1f));
                float mu = Mathf.Cos(lat); // crude daily-mean proxy

                float albedo = isOcean[i] ? prm.oceanAlbedo : prm.landAlbedo;
                float Qsw = prm.solarConstant * Mathf.Max(0.2f, 0.7f * mu) * (1f - albedo);
                float Qlw = prm.olrLinear * (T[i] - 255f);
                float groundT = prm.initialTempK - prm.lapseRate * Mathf.Max(0f, elev[i]);
                float Qsurf = prm.surfaceExchange * (groundT - T[i]);

                float net = Qsw - Qlw + Qsurf; // W/m^2
                dT_dt[i] += net / (prm.rhoAir * prm.cpAir * prm.activeLayerHeight);
            }
        }

        private void DiagnosePressureAndPGF(float[] TforPressure)
        {
            const float alpha = 150f; // Pa/K toy factor
            for (int i = 0; i < N; i++)
                p[i] = Mathf.Max(prm.minPressure, 1.0e5f + alpha * (TforPressure[i] - 288f));

            if (nbr[0] == null) return;

            for (int i = 0; i < N; i++)
            {
                var ni = nbr[i];
                if (ni == null || ni.Count == 0) continue;

                Vector3 grad = Vector3.zero;
                foreach (var j in ni)
                {
                    float dp = p[j] - p[i];
                    Vector3 tij = TangentDir(i, j);
                    float invL = 1f / Mathf.Max(1f, GreatCircleArcLength(i, j));
                    if (!Finite(invL)) invL = 0f;
                    grad += dp * invL * tij;
                }
                du_dt[i] += (-1f / prm.rhoAir) * grad;
            }
        }

        private void StepCoriolisDragDiffusion()
        {
            float Omega = prm.Omega;

            for (int i = 0; i < N; i++)
            {
                float lat = Mathf.Asin(Mathf.Clamp(up[i].y, -1f, 1f)); // swap to .z if +Z axis
                float f = 2f * Omega * Mathf.Sin(lat);

                Vector3 ui = Tangentize(u[i], up[i]);
                Vector3 coriolis = f * Vector3.Cross(up[i], ui);
                Vector3 drag = -prm.linearDrag * ui;

                du_dt[i] += coriolis + drag;
            }

            if (prm.windDiffusivity > 0f && nbr[0] != null)
            {
                float nu = prm.windDiffusivity;
                for (int i = 0; i < N; i++)
                {
                    var ni = nbr[i];
                    if (ni == null || ni.Count == 0) continue;
                    Vector3 lap = Vector3.zero;
                    foreach (var j in ni) lap += (u[j] - u[i]);
                    du_dt[i] += (nu / Mathf.Max(1f, prm.activeLayerHeight)) * lap;
                }
            }
        }

        private void StepMoistureLocal()
        {
            for (int i = 0; i < N; i++)
            {
                float qsat = SaturationSpecificHumidity(T[i], p[i]);

                if (isOcean[i])
                {
                    float E = prm.evapCoeff * Mathf.Max(0f, qsat - q[i]);
                    dq_dt[i] += E;
                }

                if (q[i] > qsat)
                {
                    float excess = q[i] - qsat;
                    float C = prm.condenseCoeff * excess;
                    dq_dt[i] -= C;
                    cloud[i] += 0.7f * C;
                    rain[i] += 0.3f * C;
                }
            }
        }

        private void StepOrographicRain()
        {
            if (nbr[0] == null) return;

            for (int i = 0; i < N; i++)
            {
                var ni = nbr[i];
                if (ni == null || ni.Count == 0) continue;

                Vector3 gradH = Vector3.zero;
                foreach (var j in ni)
                {
                    float dh = elev[j] - elev[i];
                    Vector3 tij = TangentDir(i, j);
                    gradH += SafeDiv(dh, GreatCircleArcLength(i, j)) * tij;
                }

                Vector3 ui = Tangentize(u[i], up[i]);
                float w_orog = Vector3.Dot(ui, gradH);
                if (w_orog > 0f && q[i] > 0f)
                {
                    float Rr = prm.orographicCoeff * w_orog * q[i];
                    rain[i] += Rr;
                    dq_dt[i] -= Rr;
                }
            }
        }

        private void AdvectScalar(float[] s, float[] ds_dt)
        {
            if (nbr[0] == null) return;

            for (int i = 0; i < N; i++)
            {
                var ni = nbr[i];
                if (ni == null || ni.Count == 0) continue;

                Vector3 ui = Tangentize(u[i], up[i]);

                foreach (var j in ni)
                {
                    Vector3 tij = TangentDir(i, j);
                    float vel = Vector3.Dot(ui, tij); // m/s toward j
                    float sij = (vel > 0f) ? s[i] : s[j];
                    float L = GreatCircleArcLength(i, j);
                    float flux = Mathf.Abs(vel) * L / Mathf.Max(1f, area[i]); // s^-1
                    if (!Finite(flux)) flux = 0f;
                    ds_dt[i] -= flux * (s[i] - sij);
                }
            }
        }

        private void Integrate(float dt)
        {
            for (int i = 0; i < N; i++)
            {
                T[i] += dT_dt[i] * dt;
                q[i] = Mathf.Max(0f, q[i] + dq_dt[i] * dt);
                u[i] = Tangentize(u[i] + du_dt[i] * dt, up[i]);
            }
        }

        // ---------------- Utilities ----------------

        private static Vector3 Tangentize(Vector3 v, Vector3 n) => v - Vector3.Dot(v, n) * n;

        private Vector3 TangentDir(int i, int j)
        {
            Vector3 ri = pos[i];
            Vector3 rj = pos[j];

            // Great-circle tangent at i toward j: t = ri × (rj × ri)
            Vector3 t = Vector3.Cross(ri, Vector3.Cross(rj, ri));
            float n2 = t.sqrMagnitude;
            if (!(n2 > 1e-20f)) return Vector3.zero; // avoid normalize(0)
            return t * (1.0f / Mathf.Sqrt(n2));
        }

        private float GreatCircleArcLength(int i, int j)
        {
            float c = Mathf.Clamp(Vector3.Dot(pos[i], pos[j]), -1f, 1f);
            float ang = Mathf.Acos(c);
            // Guard: if two positions collapse numerically, enforce a small length
            float L = ang * R;
            return (Finite(L) && L > 1e-3f) ? L : 1e-3f; // meters
        }

        private float SaturationSpecificHumidity(float Tkelvin, float P)
        {
            float es = prm.satA * Mathf.Exp(-prm.satB / Mathf.Max(200f, Tkelvin)); // Pa
            es = Mathf.Min(0.5f * P, es);
            float qsat = 0.622f * es / Mathf.Max(1f, (P - 0.378f * es));
            return Mathf.Clamp(qsat, 0f, 0.05f);
        }

        // ---------------- Public getters ----------------
        public float[] GetTemperature() => T;
        public float[] GetSpecificHumidity() => q;
        public float[] GetPressure() => p;
        public Vector3[] GetWind() => u;
        public float[] GetRainInstant() => rain;
        public float[] GetRainAccum() => rainAccum;

        private void OnDrawGizmos()
        {
            // REQUIRE: pos[], u[], N initialized by WeatherSystem
            if (!showWindGizmos || u == null || pos == null || N == 0) return;

            // If you already have gizmoSphereRadius from earlier, we’ll honor it; else default to 1
            float rDraw = (gizmoSphereRadius > 0f) ? gizmoSphereRadius : 1f;
            int step = Mathf.Max(1, gizmoEveryNth);

            // Work in world space for drawing
            var tr = transform;

            for (int i = 0; i < N; i += step)
            {
                // Position a tiny bit above the mesh (use your cached unit pos[])
                Vector3 rLocalUnit = pos[i];            // we normalized when bootstrapping
                Vector3 p0 = tr.TransformPoint(rLocalUnit * rDraw);

                // Tangent wind at this cell (already in local sim space)
                Vector3 v = u[i] - Vector3.Dot(u[i], rLocalUnit) * rLocalUnit;
                float speed = v.magnitude;
                if (speed < 1e-6f) continue;

                // Map speed (m/s) -> visible world length
                float len = Mathf.Min((speed / 100f) * gizmoLenPer100ms, gizmoMaxLen);
                Vector3 dirWorld = tr.TransformVector(v.normalized);
                Vector3 p1 = p0 + dirWorld * len;

                // Color by speed
                float t = Mathf.InverseLerp(0f, Mathf.Max(1e-3f, gizmoColorMaxSpeed), speed);
                Gizmos.color = Color.Lerp(gizmoWindLow, gizmoWindHigh, t);

                // Shaft
                Gizmos.DrawLine(p0, p1);

                // Arrow head
                Vector3 upWorld = tr.TransformVector(rLocalUnit);
                Vector3 side = Vector3.Cross(upWorld, dirWorld).normalized;
                float head = len * 0.25f;
                Gizmos.DrawLine(p1, p1 - dirWorld * head + side * (head * 0.6f));
                Gizmos.DrawLine(p1, p1 - dirWorld * head - side * (head * 0.6f));
            }
        }


        private float AutoDetectSphereRadius()
        {
            // Use median magnitude of dual positions
            if (pos == null || pos.Length == 0) return 1f;
            var mags = new float[Mathf.Min(pos.Length, 4096)];
            for (int i = 0; i < mags.Length; i++) mags[i] = pos[i].magnitude;
            Array.Sort(mags);
            return mags[mags.Length / 2];
        }

        [System.Runtime.CompilerServices.MethodImpl(System.Runtime.CompilerServices.MethodImplOptions.AggressiveInlining)]
        private static bool Finite(float x) => !float.IsNaN(x) && !float.IsInfinity(x);
        [System.Runtime.CompilerServices.MethodImpl(System.Runtime.CompilerServices.MethodImplOptions.AggressiveInlining)]
        private static bool Finite(Vector3 v) => Finite(v.x) && Finite(v.y) && Finite(v.z);

        [System.Runtime.CompilerServices.MethodImpl(System.Runtime.CompilerServices.MethodImplOptions.AggressiveInlining)]
        private static float SafeDiv(float num, float den, float minDen = 1f)
        {
            float d = Mathf.Abs(den);
            if (!Finite(num) || !Finite(den) || d < minDen) return 0f;
            return num / den;
        }

        private void SanitizeState(string tag)
        {
            // Clamp absurd |u| and replace any NaN/Inf with 0 to keep sim alive
            const float Umax = 250f; // m/s hard cap (above jet-stream)
            for (int i = 0; i < N; i++)
            {
                if (!Finite(T[i])) T[i] = prm.initialTempK;
                if (!Finite(q[i]) || q[i] < 0f) q[i] = 0f;

                if (!Finite(u[i].x) || !Finite(u[i].y) || !Finite(u[i].z))
                {
                    Debug.LogWarning($"[WeatherSystem] Non-finite u at {i} during {tag}; zeroing.");
                    u[i] = Vector3.zero;
                }
                float m = u[i].magnitude;
                if (m > Umax) u[i] = u[i] * (Umax / m);
            }
        }
    }
}
