using ChaosFramework.Math;

namespace ChaosFramework.Shapes.Rigging
{
    partial class Rig
    {
        public class Influence
        {
            public readonly Matrix transform;
            public readonly float weight;

            public Influence(Matrix transform, float weight)
            {
                System.Diagnostics.Debug.Assert(weight > 0);
                this.transform = transform;
                this.weight = weight;
            }
        }
    }
}
