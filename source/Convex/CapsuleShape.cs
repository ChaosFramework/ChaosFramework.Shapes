using ChaosFramework.Math.Vectors;
using ChaosUtil.Primitives;
using static ChaosFramework.Math.Clamping;
using static ChaosFramework.Math.Exponentials;

namespace ChaosFramework.Shapes.Convex
{
    using Math;

    public class CapsuleShape : Shape<CapsuleShape>
    {
        static void CalculateTipIntersections(
            Vector3f rayOrigin,
            Vector3f rayDirection,
            Vector3f sphereCenter,
            float sphereRadSq,
            Vector3f cutPlaneNormal,
            Vector3f[] intersections,
            ref bool hasEntry,
            ref bool hasExit
            )
        {
            Vector3f rayOriginToSphereCenter = sphereCenter - rayOrigin;
            float dot = Vector3f.Dot(rayDirection, rayOriginToSphereCenter);
            float discriminator = dot * dot - rayOriginToSphereCenter.LengthSq() + sphereRadSq;

            if (discriminator > 0)
            {
                float sqrt = Sqrt(discriminator);

                if (!hasEntry)
                {
                    float d = dot - sqrt;
                    Vector3f entryPoint = rayOrigin + d * rayDirection;
                    if (hasEntry = Vector3f.Dot(entryPoint - sphereCenter, cutPlaneNormal) < 0)
                        intersections[0] = entryPoint;
                }

                if (!hasExit)
                {
                    float d = dot + sqrt;
                    Vector3f exitPoint = rayOrigin + d * rayDirection;
                    if (hasExit = Vector3f.Dot(exitPoint - sphereCenter, cutPlaneNormal) < 0)
                        intersections[1] = exitPoint;
                }
            }
        }

        public Vector3f relativeA, relativeB;

        float scaledRadius;

        public float radius { get; private set; }
        public Vector3f absoluteA { get; private set; }
        public Vector3f absoluteB { get; private set; }

        CapsuleShape() { }

        public CapsuleShape(Vector3f relativeA, Vector3f relativeB, float radius)
        {
            this.relativeA = relativeA;
            this.relativeB = relativeB;
            this.radius = radius;
        }

        public override Vector3f Support(Vector3f searchDirection)
        {
            Vector3f p = Vector3f.Dot(absoluteA, searchDirection) > Vector3f.Dot(absoluteB, searchDirection) ? absoluteA : absoluteB;
            return p + scaledRadius * Vector3f.Normalize(searchDirection);
        }

        internal override void ForceUpdate(Matrix transform)
        {
            Vector3f[] transformed = Vector3f.TransformCoordinate(
                new Vector3f[] {
                    relativeA,
                    relativeB,
                    new Vector3f(0, 0.5f, 0),
                    new Vector3f(0, -0.5f, 0)
                    },
                transform
                );

            absoluteA = transformed[0];
            absoluteB = transformed[1];
            scaledRadius = (transformed[2] - transformed[3]).Length() * radius;
            hullRadius = scaledRadius + (absoluteA - absoluteB).Length();
            transformedHullCenter = (absoluteA + absoluteB) * 0.5f;

            Vector3f min = absoluteA.Min(absoluteB);
            Vector3f max = absoluteA.Max(absoluteB);
            boundingBoxLow = min - scaledRadius;
            boundingBoxHigh = max + scaledRadius;
        }

        public override Vector3f GetRandomPositionInShape()
            => absoluteA + (absoluteB - absoluteA) * Random.instance.Rnd()
             + Random.instance.RndVector3(scaledRadius * Random.instance.Rnd());

        public override Vector3f GetRandomPositionOnShape()
        {
            Vector3f d = absoluteB - absoluteA;
            float len = d.Length();
            Vector3f normal = d / len;

            Vector3f radialDirection = Random.instance.RndVector3(scaledRadius);
            float dot = Vector3f.Dot(radialDirection, normal);

            // These area values are divided by 2 * PI * scaledRadius to avoid unnecessary multiplication
            float sphereSurfaceArea = 2 * scaledRadius;
            float totalSurfaceArea = len + sphereSurfaceArea;
            if (Random.instance.RndBool(sphereSurfaceArea / totalSurfaceArea))
                return (dot < 0 ? absoluteA : absoluteB) + radialDirection;
            else
            {
                radialDirection -= dot * normal;
                radialDirection.Normalize();
                return absoluteA + Random.instance.Rnd() * d + scaledRadius * radialDirection;
            }
        }

        protected override CapsuleShape CloneSpecializedTransformed()
        {
            CapsuleShape clone = new CapsuleShape();
            clone.hullRadius = hullRadius;
            clone.boundingBoxHigh = boundingBoxHigh;
            clone.boundingBoxLow = boundingBoxLow;
            clone.transformedHullCenter = transformedHullCenter;
            clone.absoluteA = absoluteA;
            clone.absoluteB = absoluteB;
            clone.scaledRadius = scaledRadius;
            return clone;
        }

        protected override CapsuleShape CloneSpecialized()
        {
            CapsuleShape clone = CloneTransformed();
            clone.relativeA = relativeA;
            clone.relativeB = relativeB;
            clone.radius = radius;
            return clone;
        }

        protected override void ApplyTransformedData()
        {
            relativeA = absoluteA;
            relativeB = absoluteB;
            radius = scaledRadius;
        }

        public override Vector3f[] CheckLine(Vector3f rayOrigin, Vector3f rayDirection)
        {
            float offsetOnAxis, offsetOnRay;
            Projection.LineOnLine(
                absoluteA,
                absoluteB,
                rayOrigin,
                rayOrigin + rayDirection,
                out offsetOnAxis,
                out offsetOnRay
                );
            Vector3f closestOnAxisToRay = absoluteA + (absoluteB - absoluteA) * Clamp(offsetOnAxis, 0, 1);
            Vector3f closestOnRayToAxis = rayOrigin + rayDirection * Max(offsetOnRay, 0);


            Vector3f axis = absoluteB - absoluteA;

            // check if line is too far from capsule
            float distRayAxisSq = (closestOnAxisToRay - closestOnRayToAxis).LengthSq();
            float radSq = scaledRadius * scaledRadius;
            if (distRayAxisSq > radSq)
                return Array<Vector3f>.empty;

            // compensate ray to axis angle
            rayDirection.Normalize();
            float axisLengthSq = axis.LengthSq();
            Vector3f normalizedAxis = axis / Sqrt(axisLengthSq);
            float stretch = Vector3f.Cross(normalizedAxis, rayDirection).Length();

            Vector3f[] intersections = new Vector3f[2];
            float cylinderPenetrationDepth = Sqrt(radSq - distRayAxisSq);

            bool hasEntry = false, hasExit = false;
            float offsetFromProjection = cylinderPenetrationDepth / stretch;

            // check cylinder entry point
            Vector3f entryPoint = closestOnRayToAxis - offsetFromProjection * rayDirection;
            float entryDot = Vector3f.Dot(entryPoint - absoluteA, axis);
            if (entryDot >= 0 && entryDot <= axisLengthSq)
            {
                hasEntry = true;
                intersections[0] = entryPoint;
            }

            // check cylinder exit point
            Vector3f exitPoint = closestOnRayToAxis + offsetFromProjection * rayDirection;
            float exitDot = Vector3f.Dot(exitPoint - absoluteA, axis);
            if (exitDot >= 0 && exitDot <= axisLengthSq)
            {
                hasExit = true;
                intersections[1] = exitPoint;
            }

            // ray only hits the cylinder, not the tips
            if (hasEntry && hasExit)
                return intersections;

            // check A half sphere intersections
            CalculateTipIntersections(rayOrigin, rayDirection, absoluteA, radSq, axis, intersections, ref hasEntry, ref hasExit);
            if (hasEntry && hasExit)
                return intersections;

            // check B half sphere intersections
            CalculateTipIntersections(rayOrigin, rayDirection, absoluteB, radSq, -axis, intersections, ref hasEntry, ref hasExit);
            if (hasEntry && hasExit)
                return intersections;

            // rounding errors may cause "tangential" cases
            return hasEntry
                ? new[] { intersections[0] }
                : hasExit
                    ? new[] { intersections[1] }
                    : Array<Vector3f>.empty;
        }
    }
}
