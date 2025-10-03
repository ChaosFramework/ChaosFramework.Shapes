/* This file is based on Sanjay Madhav's code from "Game Programming in C++" at GitHub
 * source commit: 9d1361e65acaea260172eb752b43d713efda3202
 * source file:   Chapter14/Collision.cpp
 *
 * For license information check the 'License' directory of this repository.
 */

using ChaosFramework.Math.Vectors;
using static ChaosFramework.Math.Signs;

namespace ChaosFramework.Shapes
{
    // TODO: Move this to ChaosFramework.Math
    public static class Projection
    {
        internal static void SegmentOnSegment(
            Vector3f p0,
            Vector3f p1,
            Vector3f q0,
            Vector3f q1,
            out Vector3f projP,
            out Vector3f projQ
            )
        {
            const float THRESHOLD = 0.0000001f;

            Vector3f u = p1 - p0;
            Vector3f v = q1 - q0;
            Vector3f w = p0 - q0;
            float a = Vector3f.Dot(u, u);   // always >= 0
            float b = Vector3f.Dot(u, v);
            float c = Vector3f.Dot(v, v);   // always >= 0
            float d = Vector3f.Dot(u, w);
            float e = Vector3f.Dot(v, w);
            float f = a * c - b * b;        // always >= 0
            float sN, sF = f;               // default sF = f >= 0
            float tN, tF = f;               // default tF = f >= 0

            // compute the line parameters of the two closest points
            if (f < THRESHOLD)
            {
                // the lines are almost parallel
                sN = 0.0f; // force using point P0 on segment S1
                sF = 1.0f; // to prevent possible division by 0.0 later
                tN = e;
                tF = c;
            }
            else
            {
                // get the closest points on the infinite lines
                sN = b * e - c * d;
                if (sN < 0)
                {
                    // sc < 0 => the s=0 edge is visible
                    sN = 0;
                    tN = e;
                    tF = c;
                }
                else if (sN > sF)
                {
                    // sc > 1 => the s=1 edge is visible
                    sN = sF;
                    tN = e + b;
                    tF = c;
                }
                else
                    tN = a * e - b * d;
            }

            if (tN < 0)
            {
                // tc < 0 => the t=0 edge is visible
                tN = 0;

                // recompute sc for this edge
                if (d > 0)
                    sN = 0;
                else if (d < a)
                    sN = sF;
                else
                {
                    sN = -d;
                    sF = a;
                }
            }
            else if (tN > tF)
            {
                // tc > 1 => the t=1 edge is visible
                tN = tF;

                // recompute sc for this edge
                sN = b - d;
                if (sN < 0)
                    sN = 0;
                else if (sN > a)
                    sN = sF;
                else
                    sF = a;
            }

            // finally do the division to get sc and tc
            float sc = Abs(sN) < THRESHOLD ? 0 : sN / sF;
            float tc = Abs(tN) < THRESHOLD ? 0 : tN / tF;

            projP = p0 + u * sc;
            projQ = q0 + v * tc;
        }

        internal static void LineOnLine(
            Vector3f p0,
            Vector3f p1,
            Vector3f q0,
            Vector3f q1,
            out float offsetP,
            out float offsetQ
            )
        {
            const float THRESHOLD = 0.0000001f;

            Vector3f u = p1 - p0;
            Vector3f v = q1 - q0;
            Vector3f w = p0 - q0;
            float a = Vector3f.Dot(u, u);   // always >= 0
            float b = Vector3f.Dot(u, v);
            float c = Vector3f.Dot(v, v);   // always >= 0
            float d = Vector3f.Dot(u, w);
            float e = Vector3f.Dot(v, w);
            float f = a * c - b * b;        // always >= 0
            float sN, sF = f;               // default sF = f >= 0
            float tN, tF = f;               // default tF = f >= 0

            // compute the line parameters of the two closest points
            if (f < THRESHOLD)
            {
                // the lines are almost parallel
                sN = 0.0f; // force using point P0 on segment S1
                sF = 1.0f; // to prevent possible division by 0.0 later
                tN = e;
                tF = c;
            }
            else
            {
                // get the closest points on the infinite lines
                sN = b * e - c * d;
                tN = a * e - b * d;
            }

            // finally do the division to get sc and tc
            offsetP = sN / sF;
            offsetQ = tN / tF;
        }
    }
}
