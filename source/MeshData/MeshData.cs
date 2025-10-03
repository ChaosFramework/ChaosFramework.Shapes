using ChaosFramework.Collections.Immutable;
using ChaosFramework.Math;
using ChaosFramework.Math.Vectors;
using ChaosUtil.Primitives;
using System;
using System.Linq;
using static ChaosFramework.Math.Clamping;
using static ChaosFramework.Math.Exponentials;
using static ChaosFramework.Math.Signs;
using SysCol = System.Collections.Generic;

namespace ChaosFramework.Shapes
{
    public partial class MeshData
    {
        static readonly Vector3f X_AXIS = new Vector3f(1, 0, 0);
        static readonly Vector3f Z_AXIS = new Vector3f(0, 0, 1);

        public static MeshData sprite
            => new MeshData(
                new[] {
                    new Vector3f(-0.5f, 0.5f, 0),
                    new Vector3f(0.5f, 0.5f, 0),
                    new Vector3f(-0.5f, -0.5f, 0),
                    new Vector3f(0.5f, -0.5f, 0)
                },
                new[] {
                    new Vector3f(0, 0, -1),
                    new Vector3f(0, 0, -1),
                    new Vector3f(0, 0, -1),
                    new Vector3f(0, 0, -1)
                },
                new[] {
                    new Vector4f(1, 0, 0, 1),
                    new Vector4f(1, 0, 0, 1),
                    new Vector4f(1, 0, 0, 1),
                    new Vector4f(1, 0, 0, 1)
                },
                new[] {
                    new[] {
                        new Vector2f(0, 0),
                        new Vector2f(1, 0),
                        new Vector2f(0, 1),
                        new Vector2f(1, 1)
                    }
                },
                new uint[] { 0, 1, 2, 2, 1, 3 }
            );

        public static Vector4f[] ComputeTangents(
            uint[] inds,
            Vector3f[] pos,
            Vector2f[] tex,
            Vector3f[] nor
            )
        {
            // TODO: add spec that states that this function just shouldn't be called whenever one of these is actually null
            if (pos == null || nor == null || tex == null)
                return null;

            Vector4f[] tan = new Vector4f[pos.Length];
            Vector3f[] tan1 = new Vector3f[pos.Length];
            Vector3f[] tan2 = new Vector3f[pos.Length];

            for (int i = 0; i < inds.Length; i += 3)
            {
                uint i0 = inds[i],
                     i1 = inds[i + 1],
                     i2 = inds[i + 2];

                Vector3f pos0 = pos[i0],
                         pos1 = pos[i1],
                         pos2 = pos[i2];
                Vector2f tex0 = tex[i0],
                         tex1 = tex[i1],
                         tex2 = tex[i2];

                Vector3f edge0_pos = pos1 - pos0,
                         edge1_pos = pos2 - pos0;
                Vector2f edge0_tex = tex1 - tex0,
                         edge1_tex = tex2 - tex0;

                float r = 1 / (edge0_tex.x * edge1_tex.y - edge1_tex.x * edge0_tex.y);
                if (!float.IsInfinity(r))
                {
                    Vector3f t1 = r * (edge1_tex.y * edge0_pos - edge0_tex.y * edge1_pos);
                    tan1[i0] += t1;
                    tan1[i1] += t1;
                    tan1[i2] += t1;

                    Vector3f t2 = r * (edge0_tex.x * edge1_pos - edge1_tex.x * edge0_pos);
                    tan2[i0] += t2;
                    tan2[i1] += t2;
                    tan2[i2] += t2;
                }
            }

            for (int i = 0; i < tan.Length; i++)
            {
                Vector3f n = nor[i];
                Vector3f t1 = tan1[i];

                Vector3f t = t1 - n * Vector3f.Dot(n, t1);
                if (t.LengthSq() == 0)
                    t = nor[i] == X_AXIS
                        ? Z_AXIS
                        : Vector3f.Cross(nor[i], X_AXIS);

                float dot = Vector3f.Dot(Vector3f.Cross(n, t1), tan2[i]);
                tan[i] = new Vector4f(Vector3f.Normalize(t), dot < 0 ? -1 : 1);
            }

            return tan;
        }

        public static MeshData Assemble(MeshData[] meshes, Matrix[] transforms, Vector2f[] texCoordScale)
        {
            int numTotalVerts = 0;
            int numTexCoordPairs = 0;
            int numTotalInds = 0;

            foreach (MeshData m in meshes)
            {
                numTotalVerts += m.vertexCount;
                numTexCoordPairs = Max(m.tex.length, numTexCoordPairs);
                numTotalInds += m.ind.length;
            }

            Vector3f[] pos = new Vector3f[numTotalVerts],
                       nor = new Vector3f[numTotalVerts];
            Vector4f[] tan = new Vector4f[numTotalVerts];
            Vector2f[][] tex = new Vector2f[numTexCoordPairs][];
            uint[] inds = new uint[numTotalInds];
            for (int i = 0; i < numTexCoordPairs; i++)
                tex[i] = new Vector2f[numTotalVerts];

            uint vertexOffset = 0,
                 indexOffset = 0;

            for (int meshID = 0; meshID < meshes.Length; meshID++)
            {
                MeshData m = meshes[meshID];
                for (int i = 0; i < m.ind.length; i++)
                    inds[indexOffset++] = m.ind[i] + vertexOffset;

                Vector2f texScale = meshID < texCoordScale.Length
                                    ? texCoordScale[meshID]
                                    : new Vector2f(1, 1);

                bool untransformed = meshID >= transforms.Length;

#pragma warning disable ChaosCC0100 // use var here, as type changes in debug builds
                var unsafePosArray = m.pos.GetUnsafeUnderlyingArray();
                var unsafeNorArray = m.nor.GetUnsafeUnderlyingArray();
                var unsafeTanArray = m.tan.GetUnsafeUnderlyingArray();
#pragma warning restore ChaosCC0100

                Vector3f[] transformedPos = untransformed
                                          ? unsafePosArray
                                          : Vector3f.TransformCoordinate(unsafePosArray, transforms[meshID]);

                Vector3f[] transformedNor = untransformed
                                          ? unsafeNorArray
                                          : Vector3f.TransformNormal(unsafeNorArray, transforms[meshID]);

                Vector4f[] transformedTan;
                if (untransformed)
                    transformedTan = unsafeTanArray;
                else
                {
                    Matrix tanTransform = transforms[meshID];
                    tanTransform.m30 = tanTransform.m31 = tanTransform.m32 = 0;
                    transformedTan = Vector4f.Transform(unsafeTanArray, tanTransform);
                }

                Array.Copy(transformedPos, 0, pos, vertexOffset, m.vertexCount);
                Array.Copy(transformedNor, 0, nor, vertexOffset, m.vertexCount);
                Array.Copy(transformedTan, 0, tan, vertexOffset, m.vertexCount);
                for (int i = 0; i < m.vertexCount; i++, vertexOffset++)
                    for (int k = 0; k < m.tex.length; k++)
                        tex[k][vertexOffset] = Vector2f.ComponentWiseMul(m.tex[k][i], texScale);
            }

            return new MeshData(pos, nor, tan, tex, inds);
        }

        public static MeshData NormalizeCube(MeshData data, float borderLength = 0.5f)
        {
            float invMax = borderLength / Max(
                Abs(data.bounds.high.x),
                Abs(data.bounds.high.y),
                Abs(data.bounds.high.z),
                Abs(data.bounds.low.x),
                Abs(data.bounds.low.y),
                Abs(data.bounds.low.z)
                );

            Vector3f[] pos = new Vector3f[data.pos.length];
            for (int i = 0; i < data.pos.length; i++)
                pos[i] *= invMax;

            return new MeshData(data.flags, pos, data.nor, data.tan, data.tex, data.ind, data.customData);
        }

        public static MeshData NormalizeSphere(MeshData data, float radius = 0.5f)
        {
            float lenSq = 0;
            foreach (Vector3f v in data.pos)
                lenSq = Max(lenSq, v.LengthSq());

            float invMax = radius / Sqrt(lenSq);

            Vector3f[] pos = new Vector3f[data.pos.length];
            for (int i = 0; i < data.pos.length; i++)
                pos[i] *= invMax;

            return new MeshData(data.flags, pos, data.nor, data.tan, data.tex, data.ind, data.customData);
        }

        public static void ComputeHullRadius(SysCol.IEnumerable<Vector3f> pos, ref Bounds3f bounds, ref float hullRadius)
        {
            hullRadius = 0;
            bounds = new Bounds3f();

            foreach (Vector3f p in pos)
            {
                hullRadius = Max(hullRadius, p.LengthSq());
                bounds.Expand(p);
            }

            hullRadius = Sqrt(hullRadius);
        }

        static ImmutableArray<Vector2f> MakeImmutable(Vector2f[] tex)
            => tex;

        public readonly MeshLoadFlags flags;
        public readonly ImmutableArray<Vector3f> pos, nor;
        public readonly ImmutableArray<Vector4f> tan;
        public readonly ImmutableArray<ImmutableArray<Vector2f>> tex;
        public readonly ImmutableArray<uint> ind;
        public readonly ImmutableArray<CustomStream> customData;
        public readonly float hullRadius;
        public readonly Bounds3f bounds;

        public int numTexCoordPairs => tex == null ? 0 : tex.length;
        public int faceCount => ind == null ? 0 : ind.length / 3;
        public int vertexCount => pos == null ? 0 : pos.length;

        public MeshData(Vector3f[] pos, Vector3f[] nor, Vector4f[] tan, Vector2f[][] tex, uint[] inds)
            : this(0, pos, nor, tan, tex, inds)
        { }

        public MeshData(Vector3f[] pos, Vector3f[] nor, uint[] inds)
            : this(0, pos, nor, null, null, inds)
        { }

        public MeshData(Vector3f[] pos, Vector3f[] nor, Vector2f[][] tex, uint[] inds)
            : this(0, pos, nor, ComputeTangents(inds, pos, tex[0], nor), tex, inds)
        { }

        public MeshData(Vector3f[] pos, Vector3f[] nor, Vector2f[] tex, uint[] inds)
            : this(0, pos, nor, ComputeTangents(inds, pos, tex, nor), new Vector2f[][] { tex }, inds)
        { }

        public MeshData(
            MeshLoadFlags flags,
            Vector3f[] pos,
            Vector3f[] nor,
            Vector4f[] tan,
            Vector2f[][] tex,
            uint[] ind,
            params CustomStream[] custom
            ) : this(
                flags,
                pos,
                nor,
                tan,
                tex == null ? null : new ImmutableArray<ImmutableArray<Vector2f>>(tex.Select(MakeImmutable).ToArray()),
                ind,
                custom
                )
        { }

        public MeshData(
            MeshLoadFlags flags,
            ImmutableArray<Vector3f> pos,
            ImmutableArray<Vector3f> nor,
            ImmutableArray<Vector4f> tan,
            ImmutableArray<ImmutableArray<Vector2f>> tex,
            ImmutableArray<uint> ind,
            ImmutableArray<CustomStream> custom
            )
        {
            this.flags = flags;
            this.pos = pos;
            this.nor = nor;
            this.tan = tan;
            this.tex = tex;
            this.ind = ind;

            System.Collections.Generic.HashSet<string> streamSemantics = new System.Collections.Generic.HashSet<string>();
            foreach (CustomStream stream in custom)
                foreach (string semantic in stream.semantics)
                    if (!streamSemantics.Add(semantic))
                        throw new ArgumentException("Semantics must be unique.");

            customData = custom;
            ComputeHullRadius(pos, ref bounds, ref hullRadius);
        }

        public override int GetHashCode()
            => HashCode.Combine(vertexCount, faceCount, flags);

        public override bool Equals(object obj)
            => Equals(obj as MeshData);

        public bool Equals(MeshData compare)
            => compare != null
            && flags == compare.flags
            && pos.Equals(compare.pos)
            && nor.Equals(compare.nor)
            && tan.Equals(compare.tan)
            && tex.Equals(compare.tex)
            && ind.Equals(compare.ind)
            // TODO: this is missing custom streams
            ;

        public static bool operator ==(MeshData a, MeshData b)
            => ((object)a == null ^ (object)b == null) ? false : ((object)a == null ? (object)b == null : a.Equals(b));

        public static bool operator !=(MeshData a, MeshData b) => !(a == b);
    }
}
