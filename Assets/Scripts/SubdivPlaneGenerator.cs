// SubdividedPlane.cs
using UnityEngine;

[ExecuteInEditMode]
[RequireComponent(typeof(MeshFilter), typeof(MeshRenderer))]
public class SubdividedPlane : MonoBehaviour
{
    [Min(1)] public int resolution = 100; // number of quads along one axis
    public float size = 10f;              // world size (edge length)

    void OnValidate() => Build();

    void Awake() => Build();

    public void Build()
    {
        var mesh = new Mesh();
        mesh.indexFormat = UnityEngine.Rendering.IndexFormat.UInt32; // allow >65k verts

        int vertsPerLine = resolution + 1;
        Vector3[] verts = new Vector3[vertsPerLine * vertsPerLine];
        Vector2[] uvs = new Vector2[verts.Length];
        int[] tris = new int[resolution * resolution * 6];

        float step = size / resolution;
        for (int y = 0; y <= resolution; y++)
        {
            for (int x = 0; x <= resolution; x++)
            {
                int i = y * vertsPerLine + x;
                verts[i] = new Vector3(x * step - size * 0.5f, 0, y * step - size * 0.5f);
                uvs[i] = new Vector2((float)x / resolution, (float)y / resolution);
            }
        }

        int t = 0;
        for (int y = 0; y < resolution; y++)
        {
            for (int x = 0; x < resolution; x++)
            {
                int i = y * vertsPerLine + x;

                tris[t++] = i;
                tris[t++] = i + vertsPerLine;
                tris[t++] = i + 1;

                tris[t++] = i + 1;
                tris[t++] = i + vertsPerLine;
                tris[t++] = i + vertsPerLine + 1;
            }
        }

        mesh.vertices = verts;
        mesh.uv = uvs;
        mesh.triangles = tris;
        mesh.RecalculateNormals();

        var mf = GetComponent<MeshFilter>();
        mf.sharedMesh = mesh;
    }
}
