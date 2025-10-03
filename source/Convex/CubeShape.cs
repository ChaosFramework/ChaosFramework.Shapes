using ChaosFramework.Math.Vectors;
using ChaosUtil.Primitives;

namespace ChaosFramework.Shapes.Convex
{
    public class CubeShape : MeshShape
    {
        private CubeShape(bool _)
            : base(Array<Vector3f>.empty, false)
        { }

        public CubeShape(float rad = 0.5f) : base(
            new[] {
                new Vector3f( rad,  rad,  rad),
                new Vector3f( rad,  rad, -rad),
                new Vector3f( rad, -rad,  rad),
                new Vector3f( rad, -rad, -rad),
                new Vector3f(-rad,  rad,  rad),
                new Vector3f(-rad,  rad, -rad),
                new Vector3f(-rad, -rad,  rad),
                new Vector3f(-rad, -rad, -rad),
            }, false)
        { }

        protected override MeshShape CloneSpecializedTransformed()
        {
            CubeShape clone = new CubeShape(true);
            clone.transformedVerts = (Vector3f[])transformedVerts.Clone();
            clone.hullRadius = hullRadius;
            clone.boundingBoxHigh = boundingBoxHigh;
            clone.boundingBoxLow = boundingBoxLow;
            clone.transformedHullCenter = transformedHullCenter;
            return clone;
        }

        protected override MeshShape CloneSpecialized()
        {
            CubeShape clone = (CubeShape)CloneTransformed();
            clone.baseVerts = (Vector3f[])baseVerts.Clone();
            return clone;
        }
    }
}
