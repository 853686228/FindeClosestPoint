using System.Collections.Generic;
using UnityEngine;

namespace EngineTest
{
    public static class SAT
    {
        public static bool IsNotOverlap(List<Vector2> poly1, List<Vector2> poly2)
        {
            List<Vector2> normals = new List<Vector2>();
            normals.AddRange(GetPolyNormals(poly1));
            normals.AddRange((GetPolyNormals(poly2)));
            foreach (var normal in normals)
            {
                float min1 = float.MaxValue, min2 = float.MaxValue, max1 = float.MinValue, max2 = float.MinValue;
                foreach (var point in poly1)
                {
                    float val = Vector2.Dot(normal, point);
                    min1 = Mathf.Min(min1, val);
                    max1 = Mathf.Max(max1, val);
                }

                foreach (var point in poly2)
                {
                    float val = Vector2.Dot(normal, point);
                    min2 = Mathf.Min(min2, val);
                    max2 = Mathf.Max(max2, val);
                }

                if (min1 < min2)
                {
                    if (max1 < min2)
                        return true;
                }
                else
                {
                    if (max2 < min1)
                        return true;
                }
            }

            return false;
        }

        public static List<Vector2> GetPolyNormals(List<Vector2> poly)
        {
            List<Vector2> normals = new List<Vector2>();
            for (int i = 0; i < poly.Count; ++i)
            {
                Vector2 line = (poly[(i + 1) % poly.Count] - poly[i]).normalized;
                normals.Add(new Vector2(-line.y, line.x));
            }

            return normals;
        }
    }

    public class SATAlgorithm : MonoBehaviour
    {
        public Transform targetTrans1, targetTrans2;
        private List<Vector2> poly1, poly2;


        private List<Vector2> GetPoints(Transform trans)
        {
            var res = new List<Vector2>();
            for (int i = 0; i < trans.childCount; ++i)
            {
                res.Add(trans.GetChild(i).position);
            }

            return res;
        }

        private void OnDrawGizmos()
        {
            // if (!Application.isPlaying)
            //     return;

            poly1 = GetPoints(targetTrans1);
            poly2 = GetPoints(targetTrans2);
            bool collide = !SAT.IsNotOverlap(poly1, poly2);
            if (collide)
            {
                Gizmos.color = Color.green;
                for (int i = 0; i < poly1.Count; ++i)
                {
                    if (i == poly1.Count - 1)
                        Gizmos.DrawLine(poly1[i], poly1[0]);
                    else
                        Gizmos.DrawLine(poly1[i], poly1[i + 1]);
                }

                for (int i = 0; i < poly2.Count; ++i)
                {
                    if (i == poly2.Count - 1)
                        Gizmos.DrawLine(poly2[i], poly2[0]);
                    else
                        Gizmos.DrawLine(poly2[i], poly2[i + 1]);
                }
            }
            else
            {
                Gizmos.color = Color.red;
                for (int i = 0; i < poly1.Count; ++i)
                {
                    if (i == poly1.Count - 1)
                        Gizmos.DrawLine(poly1[i], poly1[0]);
                    else
                        Gizmos.DrawLine(poly1[i], poly1[i + 1]);
                }

                Gizmos.color = Color.cyan;
                for (int i = 0; i < poly2.Count; ++i)
                {
                    if (i == poly2.Count - 1)
                        Gizmos.DrawLine(poly2[i], poly2[0]);
                    else
                        Gizmos.DrawLine(poly2[i], poly2[i + 1]);
                }
            }
        }
    }
}