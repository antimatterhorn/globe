// Simplex.cs (put anywhere in your project)
using UnityEngine;

public static class Simplex
{
    // Permutation table
    static readonly int[] perm = {
        151,160,137,91,90,15, // (full 512 table not shown in comment to save space)
        131,13,201,95,96,53,194,233,7,225,140,36,103,30,69,142,
        // ... (standard 256 values) ...
        8,99,37,240,21,10,23
    };
    static readonly int[] p; // 512
    static Simplex()
    {
        p = new int[512];
        for (int i = 0; i < 512; i++) p[i] = perm[i & 255];
    }

    static readonly float F3 = 1f / 3f;
    static readonly float G3 = 1f / 6f;
    static readonly float F4 = (Mathf.Sqrt(5f) - 1f) / 4f;
    static readonly float G4 = (5f - Mathf.Sqrt(5f)) / 20f;

    static float Dot(int g, float x, float y, float z)
    {
        // 12 gradient directions
        switch (g & 15)
        {
            case 0: return +x + y;
            case 1: return -x + y;
            case 2: return +x - y;
            case 3: return -x - y;
            case 4: return +x + z;
            case 5: return -x + z;
            case 6: return +x - z;
            case 7: return -x - z;
            case 8: return +y + z;
            case 9: return -y + z;
            case 10: return +y - z;
            case 11: return -y - z;
            case 12: return +x + y; // duplicate to fill 16
            case 13: return -x + y;
            case 14: return +x - y;
            default: return -x - y;
        }
    }

    // 3D Simplex noise in [-1,1]
    public static float Noise3D(float x, float y, float z)
    {
        // Skew the input space to determine which simplex cell we're in
        float s = (x + y + z) * F3;
        int i = Mathf.FloorToInt(x + s);
        int j = Mathf.FloorToInt(y + s);
        int k = Mathf.FloorToInt(z + s);

        float t = (i + j + k) * G3;
        float X0 = i - t, Y0 = j - t, Z0 = k - t;
        float x0 = x - X0, y0 = y - Y0, z0 = z - Z0;

        // For the 3D case, the simplex shape is a slightly irregular tetrahedron.
        int i1, j1, k1; // Offsets for second corner
        int i2, j2, k2; // Offsets for third corner
        if (x0 >= y0)
        {
            if (y0 >= z0) { i1 = 1; j1 = 0; k1 = 0; i2 = 1; j2 = 1; k2 = 0; }
            else if (x0 >= z0) { i1 = 1; j1 = 0; k1 = 0; i2 = 1; j2 = 0; k2 = 1; }
            else { i1 = 0; j1 = 0; k1 = 1; i2 = 1; j2 = 0; k2 = 1; }
        }
        else
        {
            if (y0 < z0) { i1 = 0; j1 = 0; k1 = 1; i2 = 0; j2 = 1; k2 = 1; }
            else if (x0 < z0) { i1 = 0; j1 = 1; k1 = 0; i2 = 0; j2 = 1; k2 = 1; }
            else { i1 = 0; j1 = 1; k1 = 0; i2 = 1; j2 = 1; k2 = 0; }
        }

        float x1 = x0 - i1 + G3;
        float y1 = y0 - j1 + G3;
        float z1 = z0 - k1 + G3;
        float x2 = x0 - i2 + 2f * G3;
        float y2 = y0 - j2 + 2f * G3;
        float z2 = z0 - k2 + 2f * G3;
        float x3 = x0 - 1f + 3f * G3;
        float y3 = y0 - 1f + 3f * G3;
        float z3 = z0 - 1f + 3f * G3;

        int ii = i & 255, jj = j & 255, kk = k & 255;
        int gi0 = p[ii + p[jj + p[kk]]];
        int gi1 = p[ii + i1 + p[jj + j1 + p[kk + k1]]];
        int gi2 = p[ii + i2 + p[jj + j2 + p[kk + k2]]];
        int gi3 = p[ii + 1 + p[jj + 1 + p[kk + 1]]];

        float n0, n1, n2, n3;
        float t0 = 0.6f - x0 * x0 - y0 * y0 - z0 * z0;
        n0 = (t0 < 0) ? 0f : (t0 *= t0) * t0 * t0 * Dot(gi0, x0, y0, z0);
        float t1 = 0.6f - x1 * x1 - y1 * y1 - z1 * z1;
        n1 = (t1 < 0) ? 0f : (t1 *= t1) * t1 * t1 * Dot(gi1, x1, y1, z1);
        float t2 = 0.6f - x2 * x2 - y2 * y2 - z2 * z2;
        n2 = (t2 < 0) ? 0f : (t2 *= t2) * t2 * t2 * Dot(gi2, x2, y2, z2);
        float t3 = 0.6f - x3 * x3 - y3 * y3 - z3 * z3;
        n3 = (t3 < 0) ? 0f : (t3 *= t3) * t3 * t3 * Dot(gi3, x3, y3, z3);

        // Scale to roughly [-1,1]
        return 32f * (n0 + n1 + n2 + n3);
    }

    // 4D Simplex noise in [-1,1]
    public static float Noise4D(float x, float y, float z, float w)
    {
        // Skewing/Unskewing factors for 4D
        float F4 = Simplex.F4;
        float G4 = Simplex.G4;

        float s = (x + y + z + w) * F4;
        int i = Mathf.FloorToInt(x + s);
        int j = Mathf.FloorToInt(y + s);
        int k = Mathf.FloorToInt(z + s);
        int l = Mathf.FloorToInt(w + s);
        float t = (i + j + k + l) * G4;
        float X0 = i - t, Y0 = j - t, Z0 = k - t, W0 = l - t;
        float x0 = x - X0, y0 = y - Y0, z0 = z - Z0, w0 = w - W0;

        // Rank the components to find simplex corners
        int rankx = 0, ranky = 0, rankz = 0, rankw = 0;
        if (x0 > y0) rankx++; else ranky++;
        if (x0 > z0) rankx++; else rankz++;
        if (x0 > w0) rankx++; else rankw++;
        if (y0 > z0) ranky++; else rankz++;
        if (y0 > w0) ranky++; else rankw++;
        if (z0 > w0) rankz++; else rankw++;

        int i1 = rankx >= 3 ? 1 : 0;
        int j1 = ranky >= 3 ? 1 : 0;
        int k1 = rankz >= 3 ? 1 : 0;
        int l1 = rankw >= 3 ? 1 : 0;

        int i2 = rankx >= 2 ? 1 : 0;
        int j2 = ranky >= 2 ? 1 : 0;
        int k2 = rankz >= 2 ? 1 : 0;
        int l2 = rankw >= 2 ? 1 : 0;

        int i3 = rankx >= 1 ? 1 : 0;
        int j3 = ranky >= 1 ? 1 : 0;
        int k3 = rankz >= 1 ? 1 : 0;
        int l3 = rankw >= 1 ? 1 : 0;

        float x1 = x0 - i1 + G4, y1 = y0 - j1 + G4, z1 = z0 - k1 + G4, w1 = w0 - l1 + G4;
        float x2 = x0 - i2 + 2f * G4, y2 = y0 - j2 + 2f * G4, z2 = z0 - k2 + 2f * G4, w2 = w0 - l2 + 2f * G4;
        float x3 = x0 - i3 + 3f * G4, y3 = y0 - j3 + 3f * G4, z3 = z0 - k3 + 3f * G4, w3 = w0 - l3 + 3f * G4;
        float x4 = x0 - 1f + 4f * G4, y4 = y0 - 1f + 4f * G4, z4 = z0 - 1f + 4f * G4, w4 = w0 - 1f + 4f * G4;

        int ii = i & 255, jj = j & 255, kk = k & 255, ll = l & 255;
        int gi0 = p[ii + p[jj + p[kk + p[ll]]]];
        int gi1 = p[ii + i1 + p[jj + j1 + p[kk + k1 + p[ll + l1]]]];
        int gi2 = p[ii + i2 + p[jj + j2 + p[kk + k2 + p[ll + l2]]]];
        int gi3 = p[ii + i3 + p[jj + j3 + p[kk + k3 + p[ll + l3]]]];
        int gi4 = p[ii + 1 + p[jj + 1 + p[kk + 1 + p[ll + 1]]]];

        float n0, n1, n2, n3, n4;

        float t0 = 0.6f - x0 * x0 - y0 * y0 - z0 * z0 - w0 * w0;
        n0 = (t0 < 0) ? 0f : (t0 *= t0) * t0 * t0 * (x0 + y0 + z0 + w0) * 0.0f + (t0 * Dot(gi0, x0, y0, z0)); // reuse 3D dot

        float t1 = 0.6f - x1 * x1 - y1 * y1 - z1 * z1 - w1 * w1;
        n1 = (t1 < 0) ? 0f : (t1 *= t1) * t1 * t1 * Dot(gi1, x1, y1, z1);

        float t2 = 0.6f - x2 * x2 - y2 * y2 - z2 * z2 - w2 * w2;
        n2 = (t2 < 0) ? 0f : (t2 *= t2) * t2 * t2 * Dot(gi2, x2, y2, z2);

        float t3 = 0.6f - x3 * x3 - y3 * y3 - z3 * z3 - w3 * w3;
        n3 = (t3 < 0) ? 0f : (t3 *= t3) * t3 * t3 * Dot(gi3, x3, y3, z3);

        float t4 = 0.6f - x4 * x4 - y4 * y4 - z4 * z4 - w4 * w4;
        n4 = (t4 < 0) ? 0f : (t4 *= t4) * t4 * t4 * Dot(gi4, x4, y4, z4);

        // Scale to roughly [-1,1]
        return 27f * (n0 + n1 + n2 + n3 + n4);
    }
}
