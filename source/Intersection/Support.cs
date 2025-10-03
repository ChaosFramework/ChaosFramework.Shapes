using ChaosFramework.Math.Vectors;

namespace ChaosFramework.Shapes.Intersection
{
    public class Support
    {
        public readonly Vector3f a, b;
        public readonly Vector3f minkowski;

        public Support(Vector3f a, Vector3f b)
        {
            this.a = a;
            this.b = b;
            minkowski = a - b;
        }

        public bool IsMinkowskiEqual(Support compare)
            => minkowski == compare.minkowski;
    }
}
