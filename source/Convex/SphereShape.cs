using ChaosFramework.Collections;
using ChaosFramework.Math;
using ChaosFramework.Math.Vectors;
using ChaosUtil.Primitives;
using static ChaosFramework.Math.Exponentials;

namespace ChaosFramework.Shapes.Convex
{
    public sealed class SphereShape : Shape<SphereShape>
    {
        public static SphereShape GetHullFromPoints(Vector3f[] vertices)
        {
            if (vertices.Length == 0)
                return new SphereShape(Vector3f.EMPTY, 0);

            if (vertices.Length == 1)
                return new SphereShape(vertices[0], 0);

            vertices = new MeshShape(vertices, true).baseVerts;

            SphereShape shape = new SphereShape(vertices[0], 0);
            LinkedList<Vector3f> simplex = new LinkedList<Vector3f>(vertices[0]);
            while (Expand(shape, simplex, vertices))
                ;

            return shape;
        }

        static bool Expand(SphereShape shape, LinkedList<Vector3f> simplex, Vector3f[] verts)
        {
            float maxDistSq = 0;
            Vector3f farthest = shape.relativeCenter;
            foreach (Vector3f v in verts)
            {
                float distSq = (v - shape.relativeCenter).LengthSq();
                if (distSq > maxDistSq)
                {
                    maxDistSq = distSq;
                    farthest = v;
                }
            }

            float d = maxDistSq - shape.untransformedRadius * shape.untransformedRadius;
            if (d <= 0)
                return false;

            switch (simplex.length)
            {
                case 0:
                    throw new System.ArgumentException("SphereShape.Expand retrieved simplex with 0 vertices");

                case 1:
                    simplex.Add(farthest);
                    shape.relativeCenter = (farthest + simplex[0]) * 0.5f;
                    shape.untransformedRadius = (farthest - shape.relativeCenter).Length();
                    return true;

                case 2:
                    simplex.Add(farthest);
                    Vector3f a = shape.relativeCenter, b = (simplex.first + simplex.last) * 0.5f;
                    Vector3f sa = Vector3f.Normalize(simplex[1] - simplex.first);
                    Vector3f sb = Vector3f.Normalize(simplex.last - simplex.first);
                    Vector3f ab = b - a;
                    Vector3f dirA = ab - Vector3f.Dot(ab, sa) * sa;
                    Vector3f dirB = -ab + Vector3f.Dot(ab, sb) * sb;

                    // Ax + dirAx * f = Bx + dirBx * g => f = (Bx + dirBx * g - Ax) / dirAx
                    // Ay + dirAy * ((Bx + dirBx * g - Ax) / dirAx) = By + dirBy * g
                    // => (Bx + dirBx * g - Ax) / dirAx = (By + dirBy * g - Ay) / dirAy
                    float g = (b.x * dirA.y + (a.y - b.y) * dirA.x) / (dirA.x * dirB.y - dirA.y * dirB.x);
                    shape.relativeCenter = b + dirB * g;
                    shape.untransformedRadius = (farthest - shape.relativeCenter).Length();
                    return true;

                case 3:
                    // TODO: Can this even happen?
                    if (d > 0.001f)
                    {
                        simplex.RemoveAt(0);
                        return true;
                    }

                    return false;

                default:
                    throw new System.ArgumentException("SphereShape.Expand retrieved simplex with more than 3 vertices");

            }
        }

        public Vector3f relativeCenter;
        public float untransformedRadius;

        SphereShape() { }

        public SphereShape(Vector3f center, float radius)
            : this(center, radius, Matrix.IDENTITY)
        { }

        public SphereShape(Vector3f center, float radius, Matrix transform)
        {
            relativeCenter = center;
            untransformedRadius = radius;
            Update(transform);
        }

        public override Vector3f Support(Vector3f searchDirection)
            => transformedHullCenter + hullRadius * Vector3f.Normalize(searchDirection);

        internal override void ForceUpdate(Matrix transform)
        {
            Vector3f[] temp = Vector3f.TransformCoordinate(
                new Vector3f[] {
                    relativeCenter,
                    new Vector3f(0, 0.5f, 0),
                    new Vector3f(0, -0.5f, 0)
                },
                transform
                );

            transformedHullCenter = temp[0];
            hullRadius = (temp[1] - temp[2]).Length() * untransformedRadius;
            boundingBoxHigh = transformedHullCenter + hullRadius;
            boundingBoxLow = transformedHullCenter - hullRadius;
        }

        public override Vector3f GetRandomPositionInShape()
            => Random.instance.RndVector3(hullRadius) * Random.instance.Rnd() + transformedHullCenter;

        public override Vector3f GetRandomPositionOnShape()
            => Random.instance.RndVector3(hullRadius) + transformedHullCenter;

        protected override SphereShape CloneSpecializedTransformed()
        {
            SphereShape clone = new SphereShape();
            clone.hullRadius = hullRadius;
            clone.boundingBoxHigh = boundingBoxHigh;
            clone.boundingBoxLow = boundingBoxLow;
            clone.transformedHullCenter = transformedHullCenter;
            return clone;
        }

        protected override SphereShape CloneSpecialized()
        {
            SphereShape clone = CloneTransformed();
            clone.relativeCenter = relativeCenter;
            clone.untransformedRadius = untransformedRadius;
            return clone;
        }

        protected override void ApplyTransformedData()
        {
            untransformedRadius = hullRadius;
            relativeCenter = transformedHullCenter;
        }

        public override Vector3f[] CheckLine(Vector3f p, Vector3f direction)
        {
            Vector3f d = p - transformedHullCenter;
            float dot = -Vector3f.Dot(direction, d);
            float dirSq = direction.LengthSq();
            float hullRadiusSq = hullRadius * hullRadius;
            float discriminator = dot * dot - dirSq * (d.LengthSq() - hullRadiusSq);

            if (discriminator < 0)
                return Array<Vector3f>.empty;
            else if (discriminator == 0)
                return new Vector3f[] { p + dot / dirSq * direction };
            else
            {
                float sqrt = Sqrt(discriminator);
                float s1 = (dot - sqrt) / dirSq;
                float s2 = (dot + sqrt) / dirSq;
                return new Vector3f[] { p + s1 * direction, p + s2 * direction };
            }
        }
    }
}
