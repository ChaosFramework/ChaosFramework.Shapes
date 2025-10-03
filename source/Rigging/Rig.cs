using ChaosFramework.IO;
using ChaosFramework.IO.Streams;
using ChaosFramework.Math;
using System.IO;
using static System.Text.Encoding;
using SysCol = System.Collections.Generic;

namespace ChaosFramework.Shapes.Rigging
{
    public partial class Rig
    {
        const string ROOT_BONE_NAME = "%ROOT%";

        static readonly SysCol.Dictionary<string, Rig> prototypes = new SysCol.Dictionary<string, Rig>();

        public static Rig FromStreamSource(StreamSource source, string key)
        {
            Rig rig;
            if (prototypes.TryGetValue(key, out rig))
                return rig.Clone();

            using (Stream stream = source.OpenRead(key))
                return FromStream(stream);
        }

        public static Rig FromStream(Stream stream)
        {
            Rig rig = new Rig();
            using (BinaryReader rd = new BinaryReader(stream))
            {
                if (rd.Read<MeshData.MeshFormats>() != MeshData.MeshFormats.GMDL)
                    throw new InvalidDataException("Not a valid .gmdl stream");

                ushort numBones = rd.ReadUInt16();
                Bone[] bones = new Bone[numBones];
                ushort[] parentInds = new ushort[numBones];
                for (int i = 0; i < numBones; i++)
                {
                    bones[i] = new Bone(UTF8.GetString(rd.ReadBytes(rd.ReadUInt16())));
                    parentInds[i] = rd.ReadUInt16();
                }

                for (int i = 0; i < numBones; i++)
                {
                    Matrix rest = rd.Read<Matrix>();
                    bones[i].restTransform = Matrix.Invert(rest);
                    bones[i].length = rd.Read<float>();
                }

                rig.root = new Bone(ROOT_BONE_NAME);
                for (int i = 0; i < numBones; i++)
                    if (parentInds[i] == ushort.MaxValue)
                    {
                        rig.root.children.Add(bones[i]);
                        bones[i].parent = rig.root;
                    }
                    else
                    {
                        bones[parentInds[i]].children.Add(bones[i]);
                        bones[i].parent = bones[parentInds[i]];
                    }

                return rig;
            }
        }

        public Bone root;

        Rig() { }

        public void ComputeTransforms()
            => root.ComputeTransforms();

        public void ClearAnimations()
            => root.ClearInfluences();

        public Rig Clone()
        {
            Rig rig = new Rig();
            rig.root = root.Clone(null);
            return rig;
        }
    }
}
