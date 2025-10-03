using System;

namespace ChaosFramework.Shapes
{
    partial class MeshData
    {
        public abstract class CustomStream
        {
            public readonly string[] semantics;

            internal CustomStream(params string[] semantics)
            {
                this.semantics = semantics;
            }

            // TODO: return a readonly array view
            public abstract Array GetElements();
            public abstract Type ElementType();
        }

        public abstract class CustomStream<T>
            : CustomStream
            where T : struct
        {
            public CustomStream(params string[] semantics)
                : base(semantics)
            { }

            public override Type ElementType() => typeof(T);
        }
    }
}
