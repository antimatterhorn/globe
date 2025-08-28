using UnityEngine;

namespace Globe.Tectonics
{
    public partial class PlateSystem
    {
        private void ApplyColorsToDual()
        {
            if (_gen == null || _gen.Dual == null) return;

            var dual = _gen.Dual;
            if (dual.DualMeshVertices == null || dual.Fans == null) return;
            if (dual.DualMeshVertices.Count == 0 || dual.Fans.Count == 0) return;

            // Find mesh target (child “Dual (tiles)”, else any MeshFilter in children)
            Transform dualGO = transform.Find("Dual (tiles)");
            MeshFilter mf = (dualGO != null) ? dualGO.GetComponent<MeshFilter>() : null;
            if (mf == null) mf = GetComponentInChildren<MeshFilter>(true);
            if (mf == null) return;

            var mesh = Application.isPlaying ? mf.mesh : mf.sharedMesh;
            if (mesh == null) return;

            int verts = dual.DualMeshVertices.Count;
            if (mesh.vertexCount != verts)
            {
                Debug.LogWarning($"[PlateSystem] Vertex count mismatch: dual {verts} vs mesh {mesh.vertexCount}. Colors not applied.");
                return;
            }

            var colors = new Color[verts];

            switch (view)
            {
                case ViewMode.Continents: ColorContinents(colors); break;
                case ViewMode.Stress: ColorStress(colors); break;
                case ViewMode.Noise: ColorNoise(colors); break;
                case ViewMode.Combined: ColorCombined(colors); break;
            }

            mesh.SetColors(colors);
        }

        private void ColorContinents(Color[] colors)
        {
            var dual = _gen.Dual;
            for (int cell = 0; cell < dual.Fans.Count; cell++)
            {
                var fan = dual.Fans[cell];
                if (fan.CenterVertex < 0) continue;

                int plate = _state.CellToPlate[cell];
                var kind = _state.PlateKinds[plate];
                Color rgba = (kind == PlateKind.Continental) ? continentalColor : oceanicColor;

                colors[fan.CenterVertex] = rgba;
                int start = fan.RimStart, count = fan.RimCount;
                if (start >= 0 && start + count <= colors.Length)
                    for (int k = 0; k < count; k++) colors[start + k] = rgba;
            }
        }

        private void ColorStress(Color[] colors)
        {
            var dual = _gen.Dual;
            for (int cell = 0; cell < dual.Fans.Count; cell++)
            {
                var fan = dual.Fans[cell];
                if (fan.CenterVertex < 0) continue;

                Color rgb;
                if (_stress != null && _stress.Length > cell)
                {
                    float s = Mathf.Clamp(_stress[cell], -1f, 1f);
                    Color stressCol = (s >= 0f) ? Color.Lerp(stressMid, stressHot, s)
                                                : Color.Lerp(stressMid, stressCold, -s);
                    rgb = Color.Lerp(stressMid, stressCol, stressTint);
                }
                else rgb = stressMid;

                colors[fan.CenterVertex] = rgb;

                int start = fan.RimStart, count = fan.RimCount;
                if (start >= 0 && start + count <= colors.Length)
                    for (int k = 0; k < count; k++) colors[start + k] = rgb;
            }
        }

        private void ColorNoise(Color[] colors)
        {
            var dual = _gen.Dual;
            for (int cell = 0; cell < dual.Fans.Count; cell++)
            {
                var fan = dual.Fans[cell];
                if (fan.CenterVertex < 0) continue;

                float t = 0.5f; // default mid if noise missing
                if (_noise01 != null && _noise01.Length > cell) t = Mathf.Clamp01(_noise01[cell]);

                Color rgb = Color.Lerp(noiseLowColor, noiseHighColor, t);

                colors[fan.CenterVertex] = rgb;

                int start = fan.RimStart, count = fan.RimCount;
                if (start >= 0 && start + count <= colors.Length)
                    for (int k = 0; k < count; k++) colors[start + k] = rgb;
            }
        }

        private void ColorCombined(Color[] colors)
        {
            var dual = _gen.Dual;
            for (int cell = 0; cell < dual.Fans.Count; cell++)
            {
                var fan = dual.Fans[cell];
                if (fan.CenterVertex < 0) continue;

                float s = (_stress != null && _stress.Length > cell) ? _stress[cell] * stressScale : 0f; // ±
                float n01 = (_noise01 != null && _noise01.Length > cell) ? _noise01[cell] : 0.5f;
                float n = (n01 * 2f - 1f) * noiseScale; // zero-centered

                int plate = _state.CellToPlate[cell];
                bool isContinental = (_state.PlateKinds[plate] == PlateKind.Continental);

                // Continental: value = oceanHeight + noise + stress
                // Oceanic:     value = max(0 + noise + stress, oceanHeight)
                float value = isContinental ? (oceanHeight + n + s)
                                            : Mathf.Max(0f + n + s, oceanHeight);

                Color rgb;
                if (value <= oceanHeight + 1e-6f)
                {
                    rgb = oceanSurfaceColor; // flat ocean
                }
                else
                {
                    float h = Mathf.Max(0f, (value - oceanHeight) * landVisualScale);
                    float t = Mathf.Clamp01(h);
                    rgb = Color.Lerp(landLowColor, landHighColor, t);
                }

                colors[fan.CenterVertex] = rgb;

                int start = fan.RimStart, count = fan.RimCount;
                if (start >= 0 && start + count <= colors.Length)
                    for (int k = 0; k < count; k++) colors[start + k] = rgb;
            }
        }
    }
}
