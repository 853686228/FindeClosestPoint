using System;
using System.Collections;
using System.Collections.Generic;
using Unity.VisualScripting;
using UnityEngine;

public enum DrawMode
{
    Point2Line,
    Point2Triangle,
}

public class ClosestPoint : MonoBehaviour
{
    public Transform targetTrans;
    public Vector2  A, B, C;
    public DrawMode drawMode;

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
        private static (float, float) CulUVForLine(Vector2 p, Vector2 S, Vector2 E)
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
        private static (float, float, float) CulUVWForTriangle(Vector2 p, Vector2 A, Vector2 B, Vector2 C)
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
        /// point to triangle , clockWise input
        /// </summary>
        /// <param name="p"></param>
        /// <param name="A"></param>
        /// <param name="B"></param>
        /// <param name="C"></param>
        /// <returns></returns>
        public static Vector2 ClosestPoint(Vector2 p, Vector2 A, Vector2 B, Vector2 C)
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
    }


    private void OnDrawGizmos()
    {
        if (targetTrans == null)
            return;
        if (drawMode == DrawMode.Point2Line)
        {
            Gizmos.color = Color.red;
            Gizmos.DrawLine(A, B);
            Gizmos.color = Color.cyan;
            Gizmos.DrawSphere(targetTrans.position, 0.1f);

            Gizmos.color = Color.green;
            Gizmos.DrawSphere(ClosestPointAlgorithm.ClosestPoint(targetTrans.position, A, B), 0.1f);
        }
        else if (drawMode == DrawMode.Point2Triangle)
        {
            Gizmos.color = Color.red;
            Gizmos.DrawLine(A, B);
            Gizmos.DrawLine(B, C);
            Gizmos.DrawLine(C, A);
            Gizmos.color = Color.cyan;
            Gizmos.DrawSphere(targetTrans.position, 0.1f);

            Gizmos.color = Color.green;
            Gizmos.DrawSphere(ClosestPointAlgorithm.ClosestPoint(targetTrans.position, A, B, C), 0.1f);
        }
    }
}