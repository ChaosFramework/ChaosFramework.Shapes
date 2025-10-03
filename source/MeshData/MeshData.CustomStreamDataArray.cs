using System;
using ChaosFramework.Collections.Immutable;

namespace ChaosFramework.Shapes
{
    public partial class MeshData
    {
        public class CustomStreamDataArray<T>
            : CustomStream<T>
            where T : struct
        {
            public ImmutableArray<T> data;

            public CustomStreamDataArray(params string[] semantics)
                : this(null, semantics)
            { }

            public CustomStreamDataArray(ImmutableArray<T> data, params string[] semantics)
                : base(semantics)
            {
                this.data = data;
            }

            public override Array GetElements()
                => data.GetUnsafeUnderlyingArray();
        }
    }
}
