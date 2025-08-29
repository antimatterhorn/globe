// PlateSystem.Accessors.cs (can live at bottom of PlateSystem.cs)
using System.Collections.Generic;
using UnityEngine;

namespace Globe.Tectonics
{
    public partial class PlateSystem
    {
        public GeodesicData Geodesic => _gen?.Geodesic;
        public DualData Dual => _gen?.Dual;

        public bool IsBuilt
        {
            get
            {
                var d = _gen?.Dual;
                if (d == null || d.CellPositions == null) return false;
                int n = d.CellPositions.Count;
                return n > 0 && _state?.Elevation != null && _state.Elevation.Length == n;
            }
        }

        public int CellCount
        {
            get
            {
                var d = _gen?.Dual;
                return (d?.CellPositions != null) ? d.CellPositions.Count : 0;
            }
        }

        public Vector3 GetCellPosition(int cell)
        {
            var d = _gen?.Dual;
            if (d?.CellPositions == null || cell < 0 || cell >= d.CellPositions.Count) return Vector3.zero;
            return d.CellPositions[cell];
        }

        public float[] ElevationField => _state?.Elevation;

        public bool TryGetElevationAtCell(int cell, out float elevation)
        {
            elevation = 0f;
            var e = _state?.Elevation;
            if (e == null || cell < 0 || cell >= e.Length) return false;
            elevation = e[cell];
            return true;
        }

        // ----- Geometry hooks -----

        // Dual neighbors = geodesic vertex neighbors (1:1 mapping between dual cells and geodesic vertices)
        public IReadOnlyList<int> GetNeighbors(int cell)
        {
            var g = _gen?.Geodesic;
            if (g?.VertexNeighbors == null || cell < 0 || cell >= g.VertexNeighbors.Count) return null;
            return g.VertexNeighbors[cell];
        }

        // Optional: if you later cache per-edge arc lengths, return them here aligned to GetNeighbors order
        public float[] GetEdgeLengthsForCell(int cell)
        {
            return null; // WeatherSystem will compute great-circle arc length when this is null
        }

        // Optional: if you compute true dual cell areas, return them here (m^2). Zero/negative => use uniform area.
        public float GetCellArea(int cell) => 0f;
    }
}
