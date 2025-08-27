// GeodesicDualSphere.cs (updated to avoid mesh instantiation in Edit Mode)
using System.Collections.Generic;
using UnityEngine;
using Globe.Tectonics;
#if UNITY_EDITOR
using UnityEditor;
#endif

[ExecuteAlways]
public class GeodesicDualSphere : MonoBehaviour
{
    [Header("Resolution & Display")]
    [Range(1, 6)] public int frequency = 3;
    public float radius = 1f;
    public bool showGeodesic = true;
    public bool showDual = true;

    [Tooltip("Recreate child GameObjects next rebuild (otherwise reuse meshes).")]
    public bool forceRecreateChildren = false;

    // Children
    private GameObject geodesicGO;
    private GameObject dualGO;

    // Runtime meshes we own (assigned to sharedMesh to avoid Edit Mode instantiation)
    private Mesh geodesicMesh;
    private Mesh dualMesh;

    // Data (optional to inspect)
    public GeodesicData Geodesic;
    public DualData Dual;

#if UNITY_EDITOR
    private bool _rebuildQueued = false;
    void OnValidate() => QueueRebuild();
    private void QueueRebuild()
    {
        if (_rebuildQueued) return;
        _rebuildQueued = true;
        EditorApplication.delayCall += () =>
        {
            _rebuildQueued = false;
            if (this == null || !gameObject || !isActiveAndEnabled) return;
            Rebuild();
        };
    }
#endif

    void Reset() { Rebuild(); }
    void Start() { Rebuild(); }

    void OnDisable() { CleanupRuntimeMeshes(); }
    void OnDestroy() { CleanupRuntimeMeshes(); }

    private void CleanupRuntimeMeshes()
    {
        // Destroy meshes we created so they don't linger in memory
        if (geodesicMesh)
        {
            if (Application.isPlaying) Destroy(geodesicMesh);
            else DestroyImmediate(geodesicMesh);
            geodesicMesh = null;
        }
        if (dualMesh)
        {
            if (Application.isPlaying) Destroy(dualMesh);
            else DestroyImmediate(dualMesh);
            dualMesh = null;
        }
    }

    public void Rebuild()
    {
        if (forceRecreateChildren)
        {
            SafeClearChildren();
            geodesicGO = dualGO = null;
            forceRecreateChildren = false;
        }

        // 1) Build geometry & dual (pure)
        Geodesic = GeodesicBuilder.BuildIcosahedronGeodesic(frequency, radius);
        Dual = DualBuilder.BuildDual(Geodesic, radius);

        // 2) Ensure children & meshes
        EnsureChild(ref geodesicGO, "Geodesic (tri)");
        EnsureChild(ref dualGO, "Dual (tiles)");
        EnsureMesh(ref geodesicMesh, geodesicGO.GetComponent<MeshFilter>(), "GeodesicRuntimeMesh");
        EnsureMesh(ref dualMesh, dualGO.GetComponent<MeshFilter>(), "DualRuntimeMesh");

        // 3) Update meshes (ALWAYS via sharedMesh to avoid Edit Mode instantiation)
        if (showGeodesic) UpdateSharedMesh(geodesicMesh, Geodesic.Vertices, Geodesic.Triangles);
        ToggleRenderer(geodesicGO, showGeodesic);

        if (showDual) UpdateSharedMesh(dualMesh, Dual.DualMeshVertices, Dual.DualMeshTriangles);
        ToggleRenderer(dualGO, showDual);
    }

    private void EnsureChild(ref GameObject go, string name)
    {
        if (!go)
        {
            var t = transform.Find(name);
            if (t) go = t.gameObject;
        }
        if (!go)
        {
            go = new GameObject(name);
            go.transform.SetParent(transform, false);
            go.AddComponent<MeshFilter>();
            var mr = go.AddComponent<MeshRenderer>();
            mr.sharedMaterial = GetURPLitOrStandard();
            mr.sharedMaterial.enableInstancing = true;
        }
    }

    private void EnsureMesh(ref Mesh m, MeshFilter mf, string meshName)
    {
        if (!m)
        {
            m = new Mesh { name = meshName, hideFlags = HideFlags.DontSave };
        }
        // Assign to sharedMesh (safe in Edit Mode)
        if (mf.sharedMesh != m) mf.sharedMesh = m;
    }

    private void UpdateSharedMesh(Mesh mesh, List<Vector3> verts, List<int> tris)
    {
        mesh.Clear();
        mesh.indexFormat = (verts.Count > 65000)
            ? UnityEngine.Rendering.IndexFormat.UInt32
            : UnityEngine.Rendering.IndexFormat.UInt16;
        mesh.SetVertices(verts);
        mesh.SetTriangles(tris, 0, true);
        mesh.RecalculateNormals();
        mesh.RecalculateBounds();
    }

    private void ToggleRenderer(GameObject go, bool enable)
    {
        if (!go) return;
        var mr = go.GetComponent<MeshRenderer>();
        if (mr) mr.enabled = enable;
    }

    private void SafeClearChildren()
    {
        for (int i = transform.childCount - 1; i >= 0; --i)
            Destroy(transform.GetChild(i).gameObject); // deferred & editor-safe
    }

    private static Material GetURPLitOrStandard()
    {
        var urp = Shader.Find("Universal Render Pipeline/Lit");
        if (urp) return new Material(urp);
        var std = Shader.Find("Standard");
        return new Material(std);
    }
}
