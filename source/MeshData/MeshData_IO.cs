using ChaosFramework.Collections;
using ChaosFramework.IO;
using ChaosFramework.Math.Vectors;
using System;
using System.IO;

namespace ChaosFramework.Shapes
{
    [Flags]
    public enum MeshLoadFlags : byte
    {
        Default = 0,
        Animated = 1 << 0,
    }

    public partial class MeshData
    {
        public enum MeshFormats : uint
        {
            GMDL = 0x6C646D67
        }

        public static MeshData FromStream(string file)
        {
            using (Stream str = File.OpenRead(file))
            using (BinaryReader rd = new BinaryReader(str))
                return FromStream(rd);
        }

        public static MeshData FromStream(Stream stream)
            => FromStream(new BinaryReader(stream));

        public static MeshData FromStream(BinaryReader rd)
            => FromStream(rd, MeshFormats.GMDL);

        public static MeshData FromStream(BinaryReader rd, MeshFormats format)
        {
            switch (format)
            {
                case MeshFormats.GMDL:
                    if ((MeshFormats)rd.Read<uint>() != MeshFormats.GMDL)
                        throw new InvalidDataException("Not a valid GMDL-file");

                    MeshLoadFlags flags = rd.Read<MeshLoadFlags>();
                    byte numTexCoordPairs = rd.Read<byte>();
                    int faceCount = rd.Read<int>();
                    int vertexCount = rd.Read<int>();
                    Vector3f[] pos = new Vector3f[vertexCount];
                    Vector3f[] nor = new Vector3f[vertexCount];
                    Vector4f[] tan = new Vector4f[vertexCount];
                    Vector2f[][] tex = new Vector2f[numTexCoordPairs][];
                    for (int k = 0; k < numTexCoordPairs; ++k)
                        tex[k] = new Vector2f[vertexCount];

                    for (int i = 0; i < vertexCount; ++i)
                    {
                        pos[i] = rd.Read<Vector3f>();
                        nor[i] = rd.Read<Vector3f>();
                        for (int k = 0; k < numTexCoordPairs; ++k)
                            tex[k][i] = rd.Read<Vector2f>();
                    }

                    LinkedList<uint> inds = new LinkedList<uint>();
                    for (int i = 0; i < faceCount; ++i)
                    {
                        uint numFaceVerts = rd.Read<uint>();
                        uint first = rd.Read<uint>();
                        inds.Add(first);
                        inds.Add(rd.Read<uint>());
                        inds.Add(rd.Read<uint>());
                        for (short k = 3; k < numFaceVerts; ++k)
                        {
                            inds.Add(first);
                            inds.Add(inds[inds.length - 2]);
                            inds.Add(rd.Read<uint>());
                        }
                    }

                    if (tex.Length > 0)
                        tan = ComputeTangents(inds.ToArray(), pos, tex[0], nor);

                    return new MeshData(flags, pos, nor, tan, tex, inds.ToArray());

                default:
                    throw new NotSupportedException($"Unrecognized mesh format '{format.ToString("X8")}'.");
            }
        }

        public static void Write(BinaryWriter writer, MeshData obj)
            => obj.Write(writer);

        public void Write(string file, MeshFormats format = MeshFormats.GMDL)
        {
            using (Stream str = new FileStream(file, FileMode.Create, FileAccess.Write))
            using (BinaryWriter wr = new BinaryWriter(str))
                Write(wr, format);
        }

        public void Write(BinaryWriter writer)
            => Write(writer, MeshFormats.GMDL);

        public void Write(BinaryWriter writer, MeshFormats format)
        {
            if (pos == null || nor == null || tan == null || tex == null || ind == null)
                throw new InvalidOperationException("Can't save incomplete MeshData.");

            for (int i = 0; i < numTexCoordPairs; i++)
                if (tex[i] == null)
                    throw new InvalidOperationException("Can't save invalid MeshData.");

            switch (format)
            {
                case MeshFormats.GMDL:
                    writer.WriteAs<uint>(format);
                    writer.WriteAs(flags);
                    writer.WriteAs<byte>(numTexCoordPairs);
                    writer.WriteAs(faceCount);
                    writer.WriteAs(vertexCount);
                    for (int i = 0; i < vertexCount; ++i)
                    {
                        writer.WriteAs(pos[i]);
                        writer.WriteAs(nor[i]);
                        for (int k = 0; k < numTexCoordPairs; ++k)
                            writer.WriteAs(tex[k][i]);
                    }
                    for (int i = 0; i < faceCount; i++)
                    {
                        writer.WriteAs(3u);
                        writer.WriteAs(ind[i * 3]);
                        writer.WriteAs(ind[i * 3 + 1]);
                        writer.WriteAs(ind[i * 3 + 2]);
                    }
                    break;

                default:
                    throw new NotSupportedException($"Saving mesh format '{format.ToString("X8")}' is not supported.");
            }
        }
    }
}
