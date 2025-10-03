using ChaosFramework.Math.Vectors;
using static ChaosFramework.Math.Signs;

namespace ChaosFramework.Shapes.Intersection
{
    using Convex;

    public static class Bounds
    {
        public static bool CheckHullIntersection(Shape s1, Shape s2)
        {
            float hullSum = s1.hullRadius + s2.hullRadius;
            return hullSum * hullSum >= (s1.transformedHullCenter - s2.transformedHullCenter).LengthSq();
        }

        public static bool CheckBoundingBoxIntersection(Shape s1, Shape s2)
            => CheckBoundingBoxIntersection(s1.boundingBoxLow, s1.boundingBoxHigh, s2.boundingBoxLow, s2.boundingBoxHigh);

        public static bool CheckBoundingBoxIntersection(Vector3f low1, Vector3f high1, Vector3f low2, Vector3f high2)
        {
            Vector3f centerA = (low1 + high1) * 0.5f;
            Vector3f sizeA = (high1 - low1) * 0.5f;
            Vector3f centerB = (low2 + high2) * 0.5f;
            Vector3f sizeB = (high2 - low2) * 0.5f;
            return (Abs(centerA.x - centerB.x) <= sizeA.x + sizeB.x
                 && Abs(centerA.y - centerB.y) <= sizeA.y + sizeB.y
                 && Abs(centerA.z - centerB.z) <= sizeA.z + sizeB.z
                 );
        }
    }
}
