// Topology.cs  (replace file)
using System.Collections.Generic;
using UnityEngine;

namespace Globe.Tectonics
{
    public sealed class GeodesicData
    {
        public List<Vector3> Vertices = new();
        public List<int> Triangles = new();
        // Optional: neighbor graph (useful for kinematics convenience)
        public List<List<int>> VertexNeighbors = new();
    }

    public sealed class DualData
    {
        // Logical dual
        public List<Vector3> FaceCentroids = new();
        public List<int[]> VertexIncidentFacesOrdered = new(); // polygons as indices into FaceCentroids
        public List<int[]> DualPolygons = new();

        // Per-cell (one per geodesic vertex / dual polygon)
        public List<Vector3> CellPositions = new(); // average of polygon centroids, on-sphere (r = radius)

        // Renderable triangulation of dual polygons (triangle fans)
        public List<Vector3> DualMeshVertices = new();
        public List<int> DualMeshTriangles = new();

        // Fan mapping: one record per cell -> where its fan lives in DualMeshVertices/DualMeshTriangles
        public List<DualFan> Fans = new();
    }

    // Mapping from logical cell to triangulated fan in the render mesh
    public struct DualFan
    {
        public int CenterVertex;  // index into DualMeshVertices
        public int RimStart;      // first rim vertex index
        public int RimCount;      // number of rim vertices (polygon sides)
        public int TriStart;      // start index into DualMeshTriangles
        public int TriCount;      // number of triangle indices for this fan (multiple of 3)
    }
}