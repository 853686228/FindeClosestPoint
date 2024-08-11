using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using Unity.Burst.Intrinsics;
using Unity.VisualScripting;
using UnityEngine;

#region Data Struct

public struct Vertex
{
    public Vector2 poly1;
    public Vector2 poly2;
    public Vector2 Point;
    public float u; //用于记录simplex计算最近点的一次迭代中的重心坐标系数

    public Vertex(Vector2 p1, Vector2 p2)
    {
        poly1 = p1;
        poly2 = p2;
        Point = p1 - p2;
        u = 1;
    }

    public static implicit operator Vector2(Vertex v)
    {
        return v.Point;
    }

    public static Vector2 operator +(Vertex v1, Vertex v2)
    {
        return v1.Point + v2.Point;
    }

    public static Vector2 operator -(Vertex v1, Vertex v2)
    {
        return v1.Point - v2.Point;
    }

    public static Vector2 operator *(float val, Vertex v)
    {
        return val * v.Point;
    }

    public static Vector2 operator *(Vertex v, float val)
    {
        return val * v.Point;
    }

    // 定义取负值操作符
    public static Vector2 operator -(Vertex v)
    {
        return -v.Point;
    }
}

public struct Poly
{
    public List<Vector2> points;
    public int count;

    public Poly(List<Vector2> p)
    {
        points = new List<Vector2>(p);
        count = p.Count;
    }

    public Vector2 GetSupportPoint(Vector2 dir)
    {
        float dist = float.MinValue;
        int index = -1;
        for (int i = 0; i < count; ++i)
        {
            var val = Vector2.Dot(points[i], dir);
            if (val > dist)
            {
                dist = val;
                index = i;
            }
        }

        return points[index];
    }
}

public struct Simplex
{
    public Vertex vertexA;
    public Vertex vertexB;
    public Vertex vertexC;
    public int count; //单纯形目前的点数
    public float u, v, w;
    public float u_ab, v_ab, u_bc, v_bc, u_ca, v_ca;

    public Vector2 ClosestPoint(Vector2 p)
    {
        vertexA.u = 0;
        vertexB.u = 0;
        vertexC.u = 0;
        if (count == 1)
        {
            vertexA.u = 1;
            return vertexA;
        }
        else if (count == 2)
        {
            (u, v) = ClosestPointAlgorithm.CulUVForLine(p, vertexA, vertexB);
            if (v < 0)
            {
                vertexA.u = 1;
                return vertexA;
            }

            if (u < 0)
            {
                vertexB.u = 1;
                return vertexB;
            }

            vertexA.u = u;
            vertexB.u = v;
            return u * vertexA + v * vertexB;
        }
        else
        {
            (u_ab, v_ab) = ClosestPointAlgorithm.CulUVForLine(p, vertexA, vertexB);
            (u_bc, v_bc) = ClosestPointAlgorithm.CulUVForLine(p, vertexB, vertexC);
            (u_ca, v_ca) = ClosestPointAlgorithm.CulUVForLine(p, vertexC, vertexA);
            (u, v, w) = ClosestPointAlgorithm.CulUVWForTriangle(p, vertexA, vertexB, vertexC);

            //处理三个顶点的vonoroi区域的情况

            if (v_ab < 0 && u_ca < 0)
            {
                vertexA.u = 1;
                return vertexA;
            }

            if (v_bc < 0 && u_ab < 0)
            {
                vertexB.u = 1;
                return vertexB;
            }

            if (u_bc < 0 && v_ca < 0)
            {
                vertexC.u = 1;
                return vertexC;
            }

            //自身即在三角形内
            if ((u >= 0 && v >= 0 && w >= 0) || (u <= 0 && v <= 0 && w <= 0))
                return p;

            //p在某个边的外部， 且过p的垂线与边相交之点即为最近点
            if (u_ab > 0 && v_ab > 0 && w < 0)
            {
                vertexA.u = u_ab;
                vertexB.u = v_ab;
                return u_ab * vertexA + v_ab * vertexB;
            }

            if (u_bc > 0 && v_bc > 0 && u < 0)
            {
                vertexB.u = u_bc;
                vertexC.u = v_bc;
                return u_bc * vertexB + v_bc * vertexC;
            }

            if (u_ca > 0 && v_ca > 0 && v < 0)
            {
                vertexC.u = u_ca;
                vertexA.u = v_ca;
                return u_ca * vertexC + v_ca * vertexA;
            }

            return default;
        }
    }

    public Vertex[] GetVerts()
    {
        return new Vertex[3]
        {
            vertexA, vertexB, vertexC
        };
    }

    public List<Vector2> GetVectors()
    {
        return new List<Vector2>()
        {
            vertexA, vertexB, vertexC,
        };
    }
}

#endregion

public static class ClosestPointAlgorithm
{
    /// <summary>
    /// point to line
    /// </summary>
    /// <param name="p"></param>
    /// <param name="A"></param>
    /// <param name="B"></param>
    /// <returns></returns>
    public static Vector2 ClosestPoint(Vector2 p, Vector2 A, Vector2 B)
    {
        (var u, var v) = CulUVForLine(p, A, B);
        if (v < 0)
            return A;
        if (u < 0)
            return B;
        return u * A + v * B;
    }

    /// <summary>
    /// p = uS + vE  => u v
    /// </summary>
    /// <param name="p"></param>
    /// <param name="S"></param>
    /// <param name="E"></param>
    /// <returns></returns>
    public static (float, float) CulUVForLine(Vector2 p, Vector2 S, Vector2 E)
    {
        Vector2 SE = E - S;
        Vector2 n = SE.normalized;
        float v = ((p.x - S.x) * n.x + (p.y - S.y) * n.y) / SE.magnitude;
        float u = ((E.x - p.x) * n.x + (E.y - p.y) * n.y) / SE.magnitude;
        return (u, v);
    }

    /// <summary>
    /// p = uA + vB + wC  => u v w
    /// </summary>
    /// <param name="p"></param>
    /// <param name="A"></param>
    /// <param name="B"></param>
    /// <param name="C"></param>
    /// <returns></returns>
    public static (float, float, float) CulUVWForTriangle(Vector2 p, Vector2 A,
        Vector2 B, Vector2 C)
    {
        //AB x AC
        var abcArea2 = (B.x - A.x) * (C.y - A.y) - (C.x - A.x) * (B.y - A.y);
        //pB x pC
        var u = ((B.x - p.x) * (C.y - p.y) - (C.x - p.x) * (B.y - p.y)) / abcArea2;
        //pC x pA
        var v = ((C.x - p.x) * (A.y - p.y) - (A.x - p.x) * (C.y - p.y)) / abcArea2;
        //pA x pB
        var w = ((A.x - p.x) * (B.y - p.y) - (B.x - p.x) * (A.y - p.y)) / abcArea2;
        return (u, v, w);
    }

    /// <summary>
    /// point to triangle 
    /// </summary>
    /// <param name="p"></param>
    /// <param name="A"></param>
    /// <param name="B"></param>
    /// <param name="C"></param>
    /// <returns></returns>
    public static Vector2 ClosestPoint(Vector2 p, Vector2 A, Vector2 B,
        Vector2 C)
    {
        (var u_ab, var v_ab) = CulUVForLine(p, A, B);
        (var u_bc, var v_bc) = CulUVForLine(p, B, C);
        (var u_ca, var v_ca) = CulUVForLine(p, C, A);
        (var u, var v, var w) = CulUVWForTriangle(p, A, B, C);

        //处理三个顶点的vonoroi区域的情况
        if (v_ab < 0 && u_ca < 0) return A;
        if (v_bc < 0 && u_ab < 0) return B;
        if (u_bc < 0 && v_ca < 0) return C;

        //自身即在三角形内
        if ((u >= 0 && v >= 0 && w >= 0) || (u <= 0 && v <= 0 && w <= 0))
            return p;

        //p在某个边的外部， 且过p的垂线与边相交之点即为最近点
        if (u_ab > 0 && v_ab > 0 && w < 0)
            return u_ab * A + v_ab * B;
        if (u_bc > 0 && v_bc > 0 && u < 0)
            return u_bc * B + v_bc * C;
        if (u_ca > 0 && v_ca > 0 && v < 0)
            return u_ca * C + v_ca * A;

        return default;
    }

    /// <summary>
    /// Point to ConvexPolygon 
    /// </summary>
    /// <param name="Q"></param>
    /// <param name="convexPoly"></param>
    /// <returns></returns>
    public static (Vector2, Vector2, Vector2) ClosestPointByGJK(Poly poly1, Poly poly2)
    {
        Vector2 startDir = Vector2.up;
        Vector2 v1 = poly1.GetSupportPoint(startDir);
        Vector2 v2 = poly2.GetSupportPoint(-startDir);
        Simplex simplex = new Simplex()
        {
            vertexA = new Vertex(v1, v2),
            u = 1, count = 1,
        };

        List<Vector2> prePoly1 = new List<Vector2>(3); //上一次迭代的simplex三个点(最多)
        List<Vector2> prePoly2 = new List<Vector2>(3);
        List<Vector2> preSupportVert = new List<Vector2>
            { new Vector2(float.MinValue, float.MinValue), new Vector2(float.MinValue, float.MinValue) };
        int times = 60;
        while (true)
        {
            --times;
            if (times < 0)
                return default;

            var verts = simplex.GetVerts();
            for (int i = 0; i < simplex.count; ++i)
            {
                prePoly1.Add(verts[i].poly1);
                prePoly2.Add(verts[i].poly2);
            }

            //calculate closest point on simplex
            Vector2 P = simplex.ClosestPoint(Vector2.zero);

            if (simplex.count == 3 && simplex.u >= 0 && simplex.v >= 0 && simplex.w >= 0)
            {
                return (P, P, GetMinPenetrateDistance(simplex.GetVectors(), poly1, poly2));
            }

            if (Vector2.Dot(P, P) == 0.0f)
            {
                return (P, P, Vector2.zero);
            }

            //cull point
            if (simplex.count == 3)
            {
                //至少删一个点,如果在边的Voronoi区域，只需要删掉未参与贡献的1个点
                --simplex.count;

                //边的Voronoi区域
                if (simplex.u_ab >= 0 && simplex.v_ab >= 0 && simplex.w <= 0)
                {
                } //删C,最后1个元素，只需调整count即可
                else if (simplex.u_bc >= 0 && simplex.v_bc >= 0 && simplex.u <= 0) //删A
                {
                    simplex.vertexA = simplex.vertexB;
                    simplex.vertexB = simplex.vertexC;
                }
                else if (simplex.u_ca >= 0 && simplex.v_ca >= 0 && simplex.v <= 0) //删B
                {
                    simplex.vertexB = simplex.vertexC;
                }
                //顶点的voronoi区域, 直接把其他两个没有贡献的点都删了
                else
                {
                    --simplex.count;
                    if (simplex.v_ab < 0 && simplex.u_ca < 0)
                    {
                    } //删B C
                    else if (simplex.u_ab < 0 && simplex.v_bc < 0) //删A C
                    {
                        simplex.vertexA = simplex.vertexB;
                    }
                    else if (simplex.u_bc < 0 && simplex.v_ca < 0) //删A B
                    {
                        simplex.vertexA = simplex.vertexC;
                    }
                }
            }

            //get support point
            //如果simplex只有1个点，直接连接Q点作为朝向 
            //由于通过重心坐标计算出的P点(投影点)的u和v具有误差，所以通过直接计算该边的法线来避免
            Vector2 dir = simplex.count == 1
                ? -P
                : TripleProdFor2D(simplex.vertexB - simplex.vertexA, -simplex.vertexA);

            var supportVert1 = poly1.GetSupportPoint(dir);
            var supportVert2 = poly2.GetSupportPoint(-dir);

            //发现点和上一次迭代重复了，没有更近的点，直接返回
            for (int i = 0; i < prePoly1.Count; ++i)
            {
                if (supportVert1 == prePoly1[i] && supportVert2 == prePoly2[i])
                {
                    if (simplex.count == 1)
                        return (simplex.vertexA.poly1, simplex.vertexA.poly2, Vector2.zero);
                    else
                        return (simplex.vertexA.poly1 * simplex.vertexA.u + simplex.vertexB.poly1 * simplex.vertexB.u,
                            simplex.vertexA.poly2 * simplex.vertexA.u + simplex.vertexB.poly2 * simplex.vertexB.u,
                            Vector2.zero);
                }
            }

            //新的support Point和上次的点一样，说明进入循环了，没有更近的点
            if (supportVert1 == preSupportVert[0] && supportVert2 == preSupportVert[1])
            {
                if (prePoly1.Count == 1)
                    return (simplex.vertexA.poly1, simplex.vertexA.poly2, Vector2.zero);
                else
                    return (simplex.vertexA.poly1 * simplex.vertexA.u + simplex.vertexB.poly1 * simplex.vertexB.u,
                        simplex.vertexA.poly2 * simplex.vertexA.u + simplex.vertexB.poly2 * simplex.vertexB.u,
                        Vector2.zero);
            }

            preSupportVert.Clear();
            preSupportVert.Add(supportVert1);
            preSupportVert.Add(supportVert2);
            if (simplex.count == 1)
                simplex.vertexB = new Vertex(supportVert1, supportVert2);
            else if (simplex.count == 2)
                simplex.vertexC = new Vertex(supportVert1, supportVert2);
            simplex.count++;


            prePoly1.Clear();
            prePoly2.Clear();
        }
    }

    /// <summary>
    /// EPA算法拓展多边形，拿到最近的穿透距离
    /// </summary>
    /// <param name="simplex"></param>
    /// <param name="poly1"></param>
    /// <param name="poly2"></param>
    /// <returns></returns>
    private static Vector2 GetMinPenetrateDistance(List<Vector2> simplex, Poly poly1, Poly poly2)
    {
        do
        {
            float minDist = float.MaxValue;
            Vector2 minS = default, minE = default, closestVector = default;
            int index = -1;
            for (int i = 0; i < simplex.Count; i++)
            {
                Vector2 e = simplex[(i + 1) % simplex.Count];
                Vector2 s = simplex[i];
                var closetPoint = ClosestPoint(Vector2.zero, s, e);
                if (closetPoint.sqrMagnitude < minDist)
                {
                    minDist = closetPoint.sqrMagnitude;
                    minS = s;
                    minE = e;
                    closestVector = closetPoint;
                    index = i + 1;
                }
            }

            //计算远离远点的最近边法线
            var line = minE - minS;
            Vector2 normal = new Vector2(-line.y, line.x);
            if (Vector2.Dot(normal, -minS) > 0)
                normal *= -1;

            var p1 = poly1.GetSupportPoint(normal);
            var p2 = poly2.GetSupportPoint(-normal);
            Vector2 newPoint = p1 - p2;
            if (Vector2.Dot(newPoint, normal) < 0)
                return closestVector;
            if (newPoint == simplex[index% simplex.Count]|| newPoint == simplex[index-1])
                return closestVector;

            simplex.Insert(index, p1 - p2); //这一步很关键，如果能找到点，则该点的插入下标一定是该边两点之间
            //这是由于我们使用的闵可夫斯基差本身就是一个凸多边形，且我们之前已经判断过support点是否背离normal而朝向原点，
            //执行到这里说明一定是沿着normal方向背离原点的，且由于凸多边形每个点都代表一个方向上的最远点性质，在这个方向上能够进一步拓宽simplex的形状
            //simplex = FindConvexHull(simplex);   <-- 这一步是多于的，无须重新构造凸包
        } while (true);
    }

    /// <summary>
    /// 2D中用以计算朝向目标方向法向量的三重积
    /// </summary>
    /// <param name="start"></param>
    /// <param name="toward"></param>
    /// <returns></returns>
    private static Vector2 TripleProdFor2D(Vector2 start, Vector2 toward)
    {
        float val = start.x * toward.y - start.y * toward.x;
        return new Vector2(-start.y, start.x) * val;
    }

    private static float PolarAngle(Vector2 anchor, Vector2 point)
    {
        return Mathf.Atan2(point.y - anchor.y, point.x - anchor.x);
    }

    private static int ClockwiseTest(Vector2 center, Vector2 start, Vector2 end)
    {
        float area = (start.x - center.x) * (end.y - center.y) - (end.x - center.x) * (start.y - center.y);
        if (area > 0)
            return 1; //逆时针
        else if (area < 0)
            return -1; //顺时针
        else
            return 0; //共线
    }

    private static List<Vector2> FindConvexHull(List<Vector2> points)
    {
        Vector2 anchor = points.OrderBy(p => p.y).ThenBy(p => p.x).First();
        List<Vector2> sortedPoints =
            points.OrderBy(p => PolarAngle(anchor, p)).ThenBy(p => (p - anchor).sqrMagnitude).ToList();
        Stack<Vector2> hull = new Stack<Vector2>();
        hull.Push(sortedPoints[0]);
        hull.Push(sortedPoints[1]);
        for (int i = 2; i < sortedPoints.Count; ++i)
        {
            Vector2 next = sortedPoints[i];
            while (hull.Count >= 2 && ClockwiseTest(hull.ElementAt(1), hull.Peek(), next) <= 0)
            {
                hull.Pop();
            }
            hull.Push(next);
        }

        return hull.ToList();
    }
}


public class ClosestPoint : MonoBehaviour
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
        (var p1, var p2, var dir) = ClosestPointAlgorithm.ClosestPointByGJK(new Poly(poly1), new Poly(poly2));
        if (p1 == p2)
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

            Gizmos.color = Color.blue;
            for (int i = 0; i < poly2.Count; ++i)
            {
                if (i == poly2.Count - 1)
                    Gizmos.DrawLine(poly2[i] + dir, poly2[0] + dir);
                else
                    Gizmos.DrawLine(poly2[i] + dir, poly2[i + 1] + dir);
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


            Gizmos.color = Color.green;
            Gizmos.DrawSphere(p1, 0.2f);
            Gizmos.DrawSphere(p2, 0.2f);
            Gizmos.DrawLine(p1, p2);
        }
    }
}