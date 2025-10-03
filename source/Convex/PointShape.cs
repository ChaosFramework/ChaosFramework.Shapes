using ChaosFramework.Math;
using ChaosFramework.Math.Vectors;
using ChaosUtil.Primitives;

namespace ChaosFramework.Shapes.Convex
{
    public class PointShape : Shape<PointShape>
    {
        Vector3f pos;
        Vector3f transPos;

        public PointShape(Vector3f position)
        {
            pos = position;
            Update(Matrix.IDENTITY);
        }

        public override Vector3f GetRandomPositionInShape()
            => transPos;

        public override Vector3f GetRandomPositionOnShape()
            => transPos;

        public override Vector3f Support(Vector3f searchDirection)
            => transPos;

        internal override void ForceUpdate(Matrix transform)
        {
            transPos = Vector3f.TransformCoordinate(pos, transform);
            transformedHullCenter = transPos;
            boundingBoxHigh = transPos;
            boundingBoxLow = transPos;
            hullRadius = 0;
        }

        protected override PointShape CloneSpecializedTransformed()
        {
            PointShape clone = new PointShape(pos);
            clone.transPos = transPos;
            clone.hullRadius = 0;
            clone.boundingBoxHigh = transPos;
            clone.boundingBoxLow = transPos;
            clone.transformedHullCenter = transPos;
            return clone;
        }

        protected override PointShape CloneSpecialized()
        {
            PointShape clone = CloneTransformed();
            clone.pos = pos;
            return clone;
        }

        protected override void ApplyTransformedData()
            => pos = transPos;

        public override Vector3f[] CheckLine(Vector3f p, Vector3f direction)
            => Array<Vector3f>.empty;
    }
}
