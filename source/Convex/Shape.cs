using ChaosFramework.Math;
using ChaosFramework.Math.Vectors;
using static ChaosFramework.Math.Clamping;

namespace ChaosFramework.Shapes.Convex
{
    public abstract class Shape
    {
        public delegate Vector3f CustomEpaSupport(Vector3f searchDirection);

        public float bounce = 0.5f;
        public string name;
        public CustomEpaSupport epaSupportFunction = null;

        Matrix preTransform = Matrix.NAN;

        public Shape previousFrameShape { get; internal set; }

        public Vector3f boundingBoxLow { get; protected set; }
        public Vector3f boundingBoxHigh { get; protected set; }

        public Vector3f interFrameBoundingBoxLow { get; private set; }
        public Vector3f interFrameBoundingBoxHigh { get; private set; }

        public float hullRadius { get; internal set; }

        public Vector3f transformedHullCenter { get; internal set; }

        public Shape()
        {
            previousFrameShape = this;
            epaSupportFunction = Support;
        }

        public abstract Vector3f Support(Vector3f searchDirection);

        public Vector3f InterFrameSupport(Vector3f searchDirection)
        {
            if (previousFrameShape == null)
                return Support(searchDirection);

            Vector3f a = Support(searchDirection), b = previousFrameShape.Support(searchDirection);
            Vector3f output = Vector3f.Dot(a, searchDirection) > Vector3f.Dot(b, searchDirection) ? a : b;
            return output;
        }

        public virtual Vector3f GetFarthestVerexTo(Vector3f center)
            => transformedHullCenter + hullRadius * Vector3f.Normalize(transformedHullCenter - center);

        public virtual float GetFarthestDistanceTo(Vector3f center)
            => (transformedHullCenter - center).Length() + hullRadius;

        public virtual float GetFarthestDistSq(Vector3f center)
        {
            float farthestDist = GetFarthestDistanceTo(center);
            return farthestDist * farthestDist;
        }

        public virtual void Reset()
            => previousFrameShape = this;

        public void Update(Matrix transform, bool forceUpdate = false)
        {
            if (!forceUpdate && (transform == preTransform))
                return;

            ForceUpdate(transform);

            if (previousFrameShape != null)
            {
                interFrameBoundingBoxLow = new Vector3f(
                    Min(boundingBoxLow.x, previousFrameShape.boundingBoxLow.x),
                    Min(boundingBoxLow.y, previousFrameShape.boundingBoxLow.y),
                    Min(boundingBoxLow.z, previousFrameShape.boundingBoxLow.z)
                );
                interFrameBoundingBoxHigh = new Vector3f(
                    Max(boundingBoxHigh.x, previousFrameShape.boundingBoxHigh.x),
                    Max(boundingBoxHigh.y, previousFrameShape.boundingBoxHigh.y),
                    Max(boundingBoxHigh.z, previousFrameShape.boundingBoxHigh.z)
                );
            }
            else
            {
                interFrameBoundingBoxLow = boundingBoxLow;
                interFrameBoundingBoxHigh = boundingBoxHigh;
            }
            preTransform = transform;
        }

        public void AdvanceFrame()
        {
            // TODO: Heavily question why this is not part of Update
            previousFrameShape = CloneTypelessTransformed();
        }

        public void ApplyTransform(Matrix transform)
        {
            ForceUpdate(transform);
            ApplyTransformedData();
            ForceUpdate(Matrix.IDENTITY);
        }

        protected abstract void ApplyTransformedData();

        public abstract Shape CloneTypeless();

        public abstract Shape CloneTypelessTransformed();

        internal abstract void ForceUpdate(Matrix transform);

        public abstract Vector3f GetRandomPositionInShape();

        public abstract Vector3f GetRandomPositionOnShape();

        /// <summary>
        ///     If the line starting at point <paramref name="p"/> in <param name="direction"/> hits the surface of this shape,
        ///     this will return at least one intersection point.
        ///     <para>
        ///         If multiple intersection points are returned, they are ordered by signed distance along <paramref name="direction"/>.
        ///     </para>
        /// </summary>
        public abstract Vector3f[] CheckLine(Vector3f p, Vector3f direction);
    }

    public abstract class Shape<T>
        : Shape
        where T : Shape<T>
    {
        public sealed override Shape CloneTypeless() => Clone();
        public sealed override Shape CloneTypelessTransformed() => CloneTransformed();

        protected abstract T CloneSpecialized();
        protected abstract T CloneSpecializedTransformed();

        public T Clone()
        {
            T clone = CloneSpecialized();
            clone.name = name;
            return clone;
        }

        public T CloneTransformed()
        {
            T clone = CloneSpecializedTransformed();
            clone.name = name;
            return clone;
        }
    }
}
