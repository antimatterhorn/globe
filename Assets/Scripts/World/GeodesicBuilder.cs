using System.Collections.Generic;
using UnityEngine;
using static Globe.Tectonics.SphereMath;

namespace Globe.Tectonics
{
    public static class GeodesicBuilder
    {
        public static GeodesicData BuildIcosahedronGeodesic(int frequency, float radius)
        {
            var data = new GeodesicData();

            // Base icosahedron verts (unit)
            float t = (1f + Mathf.Sqrt(5f)) / 2f;
            var baseVerts = new List<Vector3>
            {
                new(-1,  t,  0), new( 1,  t,  0), new(-1, -t,  0), new( 1, -t,  0),
                new( 0, -1,  t), new( 0,  1,  t), new( 0, -1, -t), new( 0,  1, -t),
                new( t,  0, -1), new( t,  0,  1), new(-t,  0, -1), new(-t,  0,  1),
            };
            for (int i = 0; i < baseVerts.Count; i++) baseVerts[i] = baseVerts[i].normalized;

            int[] faces = {
                0,11,5, 0,5,1, 0,1,7, 0,7,10, 0,10,11,
                1,5,9, 5,11,4, 11,10,2, 10,7,6, 7,1,8,
                3,9,4, 3,4,2, 3,2,6, 3,6,8, 3,8,9,
                4,9,5, 2,4,11, 6,2,10, 8,6,7, 9,8,1
            };

            var verts = new List<Vector3>(baseVerts);
            var tris = new List<int>(faces);

            var midpointCache = new Dictionary<long, int>();
            for (int s = 0; s < frequency; s++)
            {
                var newTris = new List<int>(tris.Count * 4);
                for (int tIdx = 0; tIdx < tris.Count; tIdx += 3)
                {
                    int a = tris[tIdx], b = tris[tIdx + 1], c = tris[tIdx + 2];
                    int ab = GetMidpointIndex(a, b, verts, midpointCache);
                    int bc = GetMidpointIndex(b, c, verts, midpointCache);
                    int ca = GetMidpointIndex(c, a, verts, midpointCache);

                    newTris.AddRange(new int[] {
                        a, ab, ca,
                        b, bc, ab,
                        c, ca, bc,
                        ab, bc, ca
                    });
                }
                tris = newTris;
            }

            for (int i = 0; i < verts.Count; i++) verts[i] = NormalizeToRadius(verts[i], radius);

            data.Vertices = verts;
            data.Triangles = tris;
            return data;
        }

        private static int GetMidpointIndex(int i, int j, List<Vector3> verts, Dictionary<long, int> cache)
        {
            long key = EdgeKey(i, j);
            if (cache.TryGetValue(key, out int idx)) return idx;

            var mid = (verts[i] + verts[j]).normalized; // unit sphere
            idx = verts.Count;
            verts.Add(mid);
            cache[key] = idx;
            return idx;
        }
    }
}
