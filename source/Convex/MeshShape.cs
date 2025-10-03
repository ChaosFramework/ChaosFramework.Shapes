using ChaosFramework.Shapes.Intersection;
using ChaosFramework.Collections;
using ChaosFramework.Math;
using ChaosFramework.Math.Vectors;
using ChaosUtil.Primitives;
using static ChaosFramework.Math.Clamping;
using static ChaosFramework.Math.Exponentials;

namespace ChaosFramework.Shapes.Convex
{
    public class MeshShape : Shape<MeshShape>
    {
        static readonly Vector3f DOWN = new Vector3f(0, -1, 0);

        static bool LineHitsTriangle(Vector3f p, Vector3f d, Vector3f a, Vector3f b, Vector3f c, out Vector3f projection)
        {
            Vector3f normal = Vector3f.Normalize(Vector3f.Cross(b - a, c - a));
            float distance = Vector3f.Dot(p, normal) - Vector3f.Dot(a, normal);
            projection = p - d * (distance / Vector3f.Dot(d, normal));

            Vector3f prismA = Vector3f.Cross(b - a, normal);
            Vector3f prismB = Vector3f.Cross(c - b, normal);
            Vector3f prismC = Vector3f.Cross(a - c, normal);

            return Vector3f.Dot(projection, prismA) - Vector3f.Dot(a, prismA) <= 0
                && Vector3f.Dot(projection, prismB) - Vector3f.Dot(b, prismB) <= 0
                && Vector3f.Dot(projection, prismC) - Vector3f.Dot(c, prismC) <= 0;
        }

        public Vector3f[] baseVerts;
        public Vector3f[] transformedVerts;

        MeshShape() { }

        public MeshShape(Vector3f[] baseVerts, bool optimize = false)
        {
            this.baseVerts = baseVerts;
            if (optimize)
                Optimize();

            Update(Matrix.IDENTITY);
        }

        public void Optimize()
        {
            LinkedList<Vector3f> outLst = new LinkedList<Vector3f>();
            foreach (Vector3f pos in baseVerts)
                if (!outLst.Contains(pos))
                    outLst.Add(pos);

            foreach (Vector3f pos in baseVerts)
            {
                outLst.Remove(pos);
                MeshShape s1 = new MeshShape(outLst.ToArray());
                PointShape s2 = new PointShape(pos);

                if (Intersection.ConvexHull.CheckIntersection(s1, s2) == null)
                    outLst.Add(pos);
            }

            baseVerts = outLst.ToArray();
            Update(Matrix.IDENTITY);
        }

        public override Vector3f Support(Vector3f searchDirection)
        {
            if (transformedVerts == null)
                return DOWN;

            Vector3f max = new Vector3f();
            float maxDist = float.MinValue;
            foreach (Vector3f v in transformedVerts)
            {
                float dot = Vector3f.Dot(v, searchDirection);
                if (dot > maxDist)
                {
                    max = v;
                    maxDist = dot;
                }
            }

            return max;
        }

        internal override void ForceUpdate(Matrix transform)
        {
            transformedVerts = Vector3f.TransformCoordinate(baseVerts, transform);
            transformedHullCenter = Vector3f.EMPTY;
            foreach (Vector3f pos in transformedVerts)
                transformedHullCenter += pos;

            transformedHullCenter *= (1f / transformedVerts.Length);

            float radSq = 0;
            boundingBoxHigh = transformedHullCenter;
            boundingBoxLow = transformedHullCenter;
            foreach (Vector3f pos in transformedVerts)
            {
                float posSq = (pos - transformedHullCenter).LengthSq();
                if (posSq > radSq)
                    radSq = posSq;
                boundingBoxHigh = new Vector3f(Max(boundingBoxHigh.x, pos.x), Max(boundingBoxHigh.y, pos.y), Max(boundingBoxHigh.z, pos.z));
                boundingBoxLow = new Vector3f(Min(boundingBoxLow.x, pos.x), Min(boundingBoxLow.y, pos.y), Min(boundingBoxLow.z, pos.z));
            }

            hullRadius = Sqrt(radSq);
        }

        public override Vector3f GetRandomPositionInShape()
        {
            SphereShape shape = new SphereShape(Vector3f.EMPTY, 0);
            for (int i = 0; i < 100; i++)
            {
                Vector3f rndPos = Random.instance.RndVector3(hullRadius) * Random.instance.Rnd() + transformedHullCenter;
                shape.transformedHullCenter = rndPos;
                if (ConvexHull.CheckIntersection(shape, this) != null)
                    return rndPos;
            }

            return transformedHullCenter;
        }

        public override Vector3f GetRandomPositionOnShape()
        {
            const int NUM_RETRIES = 3;

            Vector3f randomDirection = Random.instance.RndVector3();
            for (int i = 0; i < NUM_RETRIES; i++)
            {
                Vector3f[] result = CheckLine(transformedHullCenter, randomDirection);
                if (result.Length > 0)
                    return result[0];

                randomDirection = Random.instance.RndVector3();
            }

            return Support(randomDirection);
        }

        protected override MeshShape CloneSpecializedTransformed()
        {
            MeshShape clone = new MeshShape();
            clone.transformedVerts = (Vector3f[])transformedVerts.Clone();
            clone.hullRadius = hullRadius;
            clone.boundingBoxHigh = boundingBoxHigh;
            clone.boundingBoxLow = boundingBoxLow;
            clone.transformedHullCenter = transformedHullCenter;
            return clone;
        }
        protected override MeshShape CloneSpecialized()
        {
            MeshShape clone = CloneTransformed();
            clone.baseVerts = (Vector3f[])baseVerts.Clone();
            return clone;
        }

        protected override void ApplyTransformedData()
        {
            for (int i = 0; i < baseVerts.Length; i++)
                baseVerts[i] = transformedVerts[i];
        }

        public override Vector3f[] CheckLine(Vector3f p, Vector3f direction)
        {
            // TODO: this is entirely broken. Fix it.
            direction.Normalize();

            Vector3f proj;
            Vector3f support0 = Support(-direction);
            Vector3f support0Projection = p + direction * Vector3f.Dot(support0 - p, direction);
            Vector3f support1 = Support(support0Projection - support0);
            if (support1.IsNaN() || support0.IsNaN() || Vector3f.Dot(support1 - support0Projection, support0 - support0Projection) >= 0)
                return new Vector3f[0];

            Vector3f support2Direction = Vector3f.Cross(direction, support1 - support0);
            if (Vector3f.Dot(support2Direction, support0Projection) < Vector3f.Dot(support2Direction, support0))
                support2Direction *= -1;

            Vector3f support2 = Support(support2Direction);
            if (support2.IsNaN() || Vector3f.Dot(support2 - p, support2Direction) <= 0)
                return new Vector3f[0];

            int numIterations = 0;
            while (true)
            {
                Vector3f normal = Vector3f.Cross(support2 - support0, support1 - support0);
                normal.Normalize();
                if (Vector3f.Dot(normal, direction) > 0)
                {
                    normal *= -1;
                    Vector3f tmp = support1;
                    support1 = support2;
                    support2 = tmp;
                }
                Vector3f newSupport = Support(normal);
                if (numIterations++ > 100
                    || newSupport == support0
                    || newSupport == support1
                    || newSupport == support2
                    || Vector3f.Dot(newSupport, normal) <= Vector3f.Dot(support0, normal) + 0.0001f)
                {
                    //We found our triangle. Compute point of impact.
                    float d = Vector3f.Dot(p, normal) - Vector3f.Dot(support0, normal);
                    Vector3f projected = p - (d / Vector3f.Dot(direction, normal)) * direction;
                    if (numIterations >= 100)
                        return new Vector3f[0];

                    //TODO: Figure out why this can happen
                    if (Intersection.ConvexHull.CheckIntersection(new SphereShape(projected, 0.02f), this) == null)
                        return new Vector3f[0];

                    return new Vector3f[] { projected };
                }
                else
                {
                    if (LineHitsTriangle(p, direction, support0, support1, newSupport, out proj))
                        support2 = newSupport;
                    else if (LineHitsTriangle(p, direction, support1, support2, newSupport, out proj))
                        support0 = newSupport;
                    else
                        support1 = newSupport;
                }
            }
        }
    }
}
