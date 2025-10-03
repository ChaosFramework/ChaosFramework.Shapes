using ChaosFramework.Collections;
using ChaosFramework.Math.Vectors;
using static ChaosFramework.Math.Clamping;

namespace ChaosFramework.Shapes.Intersection
{
    using Math;
    using ChaosFramework.Shapes.Convex;

    public static class ConvexHull
    {
        const short MAX_ITERATIONS = 30;

        public static LinkedList<Support> CheckIntersection(Shape s1, Shape s2, bool useInterFrameCollisionDetection = false)
        {
            if (useInterFrameCollisionDetection)
            {
                Vector3f proj1, proj2;
                Projection.SegmentOnSegment(
                    s1.previousFrameShape.transformedHullCenter,
                    s1.transformedHullCenter,
                    s2.previousFrameShape.transformedHullCenter,
                    s2.transformedHullCenter,
                    out proj1,
                    out proj2
                    );

                float radius = Max(s1.previousFrameShape.hullRadius, s1.hullRadius)
                             + Max(s2.previousFrameShape.hullRadius, s2.hullRadius);

                if ((proj1 - proj2).LengthSq() > radius * radius)
                    return null;
            }
            else if (!Bounds.CheckHullIntersection(s1, s2))
                return null;

            Vector3f dir = new Vector3f(1, -1, 0.5f);
            Support s = new Support(s1.InterFrameSupport(dir), s2.InterFrameSupport(-dir));
            LinkedList<Support> simplex = new LinkedList<Support>();
            simplex.Add(s);
            dir = -s.minkowski;
            for (int i = 0; i < MAX_ITERATIONS; i++)
            {
                s = new Support(s1.InterFrameSupport(dir), s2.InterFrameSupport(-dir));
                if (Vector3f.Dot(s.minkowski, dir) < 0)
                    return null;

                simplex.Insert(0, s);
                if (DoSimplex(ref simplex, ref dir))
                    return simplex;
            }

            return null;
        }

        private static bool DoSimplex(ref LinkedList<Support> simplex, ref Vector3f dir)
        {
            switch (simplex.length)
            {
                case 2:
                    return DoLine(ref simplex, ref dir);

                case 3:
                    DoTriangle(ref simplex, ref dir);
                    return false;

                case 4:
                    return DoTetrahedron(ref simplex, ref dir);

                default:
                    return false;
            }
        }

        private static bool DoLine(ref LinkedList<Support> simplex, ref Vector3f dir)
        {
            Vector3f ab = simplex[1].minkowski - simplex[0].minkowski;
            if (Vector3f.Dot(ab, -simplex[0].minkowski) >= 0)
            {
                dir = Vector3f.Cross(Vector3f.Cross(ab, -simplex[0].minkowski), ab);

                float dotProduct = Vector3f.Dot(simplex[1].minkowski, simplex[0].minkowski);
                if (dotProduct * dotProduct < ((simplex[1].minkowski - simplex[0].minkowski).LengthSq()) * -0.99f)
                    return true;
            }
            else
            {
                dir = -simplex[0].minkowski;
                simplex.RemoveAt(1);
            }
            return false;
        }

        private static void DoTriangle(ref LinkedList<Support> simplex, ref Vector3f dir)
        {
            Support a = simplex[0];
            Support b = simplex[1];
            Support c = simplex[2];

            Vector3f ab = b.minkowski - a.minkowski;
            Vector3f ac = c.minkowski - a.minkowski;

            Vector3f abcNor = Vector3f.Cross(ab, ac);

            if (Vector3f.Dot(Vector3f.Cross(abcNor, ac), -a.minkowski) > 0)
            {
                if (Vector3f.Dot(ac, -a.minkowski) > 0)
                {
                    simplex.RemoveAt(1);
                    dir = Vector3f.Cross(Vector3f.Cross(ac, -a.minkowski), ac);
                    return;
                }
                else
                    goto remove2;
            }
            else
            {
                if (Vector3f.Dot(Vector3f.Cross(ab, abcNor), -a.minkowski) > 0)
                    goto remove2;
                else
                {
                    if (Vector3f.Dot(abcNor, -a.minkowski) > 0)
                    {
                        dir = abcNor;
                        return;
                    }
                    else
                    {
                        dir = -abcNor;
                        simplex.Clear();
                        simplex.Add(a, c, b);
                        return;
                    }
                }
            }

        remove2:
            if (Vector3f.Dot(ab, -a.minkowski) > 0)
            {
                simplex.RemoveAt(2);
                dir = Vector3f.Cross(Vector3f.Cross(ab, -a.minkowski), ab);
            }
            else
            {
                simplex.Clear();
                simplex.Add(a);
                dir = -a.minkowski;
            }
        }

        private static bool DoTetrahedron(ref LinkedList<Support> simplex, ref Vector3f dir)
        {
            Support a = simplex[0];
            Support b = simplex[1];
            Support c = simplex[2];
            Support d = simplex[3];

            Vector3f ab = b.minkowski - a.minkowski;
            Vector3f ac = c.minkowski - a.minkowski;
            Vector3f ad = d.minkowski - a.minkowski;

            Vector3f abcNor = Vector3f.Cross(ab, ac);
            Vector3f acdNor = Vector3f.Cross(ac, ad);
            Vector3f adbNor = Vector3f.Cross(ad, ab);

            bool isInAbc = Vector3f.Dot(abcNor, a.minkowski) < 0;
            bool isInAdb = Vector3f.Dot(adbNor, a.minkowski) < 0;
            bool isInAcd = Vector3f.Dot(acdNor, a.minkowski) < 0;

            if (!isInAbc && !isInAdb && !isInAcd)
                return true;

            if (isInAbc)
            {
                simplex.RemoveAt(3);
                dir = abcNor;
            }
            else if (isInAdb)
            {
                simplex.RemoveAt(2);
                dir = adbNor;
            }
            else
            {
                simplex.RemoveAt(1);
                dir = acdNor;
            }

            DoTriangle(ref simplex, ref dir);

            return false;
        }
    }
}
