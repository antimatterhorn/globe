// DualBuilder.cs (fixed)
using System.Collections.Generic;
using UnityEngine;
using static Globe.Tectonics.SphereMath;

namespace Globe.Tectonics
{
    public static class DualBuilder
    {
        public static DualData BuildDual(GeodesicData g, float radius)
        {
            var outData = new DualData();
            int triCount = g.Triangles.Count / 3;

            // 1) face centroids
            var centroids = new List<Vector3>(triCount);
            for (int f = 0; f < triCount; f++)
            {
                int i0 = g.Triangles[3 * f + 0];
                int i1 = g.Triangles[3 * f + 1];
                int i2 = g.Triangles[3 * f + 2];
                Vector3 c = (g.Vertices[i0] + g.Vertices[i1] + g.Vertices[i2]) / 3f;
                centroids.Add(NormalizeToRadius(c, radius));
            }

            // 2) incident faces per vertex
            var incident = new List<List<int>>(g.Vertices.Count);
            for (int v = 0; v < g.Vertices.Count; v++) incident.Add(new List<int>());
            for (int f = 0; f < triCount; f++)
            {
                incident[g.Triangles[3 * f + 0]].Add(f);
                incident[g.Triangles[3 * f + 1]].Add(f);
                incident[g.Triangles[3 * f + 2]].Add(f);
            }

            // 3) order incident faces CCW on tangent plane & compute cell positions
            var orderedIncident = new List<int[]>(g.Vertices.Count);
            var cellPositions = new List<Vector3>(g.Vertices.Count);
            for (int v = 0; v < g.Vertices.Count; v++)
            {
                var list = incident[v];
                if (list.Count == 0)
                {
                    orderedIncident.Add(System.Array.Empty<int>());
                    cellPositions.Add(NormalizeToRadius(g.Vertices[v], radius));
                    continue;
                }

                OrthonormalBasis(g.Vertices[v], out var t1, out var t2);
                Vector3 n = g.Vertices[v].normalized;

                list.Sort((a, b) =>
                {
                    float A = AngleOnTangent(centroids[a], n, t1, t2);
                    float B = AngleOnTangent(centroids[b], n, t1, t2);
                    return A.CompareTo(B);
                });

                orderedIncident.Add(list.ToArray());

                Vector3 avg = Vector3.zero;
                foreach (var fi in list) avg += centroids[fi];
                cellPositions.Add(NormalizeToRadius(avg / list.Count, radius));
            }

            // 4) polygons (dual)
            var dualPolys = new List<int[]>(g.Vertices.Count);
            for (int v = 0; v < g.Vertices.Count; v++) dualPolys.Add(orderedIncident[v]);

            // 5) triangulate polygons into renderable mesh and record fan map
            var dVerts = new List<Vector3>();
            var dTris = new List<int>();
            var fans = new List<DualFan>(dualPolys.Count);

            for (int p = 0; p < dualPolys.Count; p++)
            {
                var poly = dualPolys[p];
                if (poly == null || poly.Length < 3)
                {
                    fans.Add(new DualFan { CenterVertex = -1, RimStart = -1, RimCount = 0, TriStart = dTris.Count, TriCount = 0 });
                    continue;
                }

                Vector3 avg = Vector3.zero;
                foreach (var fi in poly) avg += centroids[fi];
                avg = NormalizeToRadius(avg / poly.Length, radius);

                int center = dVerts.Count;
                dVerts.Add(avg);

                int rimStart = dVerts.Count;
                for (int k = 0; k < poly.Length; k++)
                    dVerts.Add(centroids[poly[k]]);

                int triStart = dTris.Count;
                for (int k = 0; k < poly.Length; k++)
                {
                    int a = center;
                    int b = rimStart + k;
                    int c = rimStart + ((k + 1) % poly.Length);
                    dTris.Add(a); dTris.Add(b); dTris.Add(c);
                }
                int fanTriCount = dTris.Count - triStart; // <-- renamed to avoid shadowing

                fans.Add(new DualFan
                {
                    CenterVertex = center,
                    RimStart = rimStart,
                    RimCount = poly.Length,
                    TriStart = triStart,
                    TriCount = fanTriCount
                });
            }

            // Optional: geodesic vertex neighbors (cell graph)
            if (g.VertexNeighbors == null || g.VertexNeighbors.Count != g.Vertices.Count)
            {
                g.VertexNeighbors = new List<List<int>>(g.Vertices.Count);
                for (int v = 0; v < g.Vertices.Count; v++) g.VertexNeighbors.Add(new List<int>());
                var seen = new HashSet<(int, int)>();
                for (int i = 0; i < g.Triangles.Count; i += 3)
                {
                    int a = g.Triangles[i], b = g.Triangles[i + 1], c = g.Triangles[i + 2];
                    AddEdge(a, b); AddEdge(b, c); AddEdge(c, a);
                }
                void AddEdge(int i, int j)
                {
                    int u = i < j ? i : j, v = i < j ? j : i;
                    if (!seen.Add((u, v))) return;
                    g.VertexNeighbors[u].Add(v);
                    g.VertexNeighbors[v].Add(u);
                }
            }

            outData.FaceCentroids = centroids;
            outData.VertexIncidentFacesOrdered = orderedIncident;
            outData.DualPolygons = dualPolys;
            outData.CellPositions = cellPositions;
            outData.DualMeshVertices = dVerts;
            outData.DualMeshTriangles = dTris;
            outData.Fans = fans;
            return outData;
        }
    }
}
