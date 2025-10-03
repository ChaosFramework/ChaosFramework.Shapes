using ChaosFramework.Collections;
using ChaosFramework.IO;
using ChaosFramework.Math;
using ChaosFramework.Math.Vectors;
using System.IO;
using System.Linq;
using static System.Text.Encoding;
using SysCol = System.Collections.Generic;

namespace ChaosFramework.Shapes.Rigging
{
    public enum FileKeyFrameType : byte
    {
        Scaling,
        Orientation,
        Translation,
    }

    public enum KeyFrameChannel : byte
    {
        ScalingX,
        ScalingY,
        ScalingZ,
        OrientationW,
        OrientationX,
        OrientationY,
        OrientationZ,
        PositionX,
        PositionY,
        PositionZ,
    }

    public class Animation
    {
        internal class KeyFrame
        {
            public float time;
            public float value;

            public KeyFrame(float time, float value)
            {
                this.time = time;
                this.value = value;
            }
        }

        static readonly int NUM_KEY_FRAME_CHANNELS = System.Enum.GetNames(typeof(KeyFrameChannel)).Length;
        static readonly int NUM_KEY_FRAME_TYPES = System.Enum.GetNames(typeof(FileKeyFrameType)).Length;

        public static SysCol.Dictionary<string, Animation> FromStream(Stream stream)
        {
            using (BinaryReader reader = new BinaryReader(stream))
            {
                SysCol.Dictionary<string, Animation> animations = new SysCol.Dictionary<string, Animation>();

                if (reader.Read<MeshData.MeshFormats>() != MeshData.MeshFormats.GMDL)
                    throw new InvalidDataException("not a valid animation file");

                uint animationLength = reader.Read<uint>();
                ushort numActions = reader.Read<ushort>();
                if (numActions == 0)
                    throw new InvalidDataException("animation file does not define actions");

                for (int _ = 0; _ < numActions; _++)
                {
                    string actionName = UTF8.GetString(reader.ReadBytes(reader.Read<short>()));
                    Animation actionAnim = new Animation(actionName);
                    ushort numAffectedBones = reader.Read<ushort>();
                    for (ushort __ = 0; __ < numAffectedBones; __++)
                    {
                        string boneName = UTF8.GetString(reader.ReadBytes(reader.Read<short>()));
                        for (FileKeyFrameType type = 0; (byte)type < NUM_KEY_FRAME_TYPES; type++)
                        {
                            int numKeyFrames = reader.ReadUInt16();
                            for (int frame = 0; frame < numKeyFrames; frame++)
                            {
                                float time = reader.Read<float>() / animationLength;
                                switch (type)
                                {
                                    case FileKeyFrameType.Scaling:
                                        Vector3f scale = reader.Read<Vector3f>();
                                        actionAnim.AddKeyFrame(KeyFrameChannel.ScalingX, boneName, time, scale.x);
                                        actionAnim.AddKeyFrame(KeyFrameChannel.ScalingY, boneName, time, scale.y);
                                        actionAnim.AddKeyFrame(KeyFrameChannel.ScalingZ, boneName, time, scale.z);
                                        break;
                                    case FileKeyFrameType.Orientation:
                                        Quaternion ori = reader.Read<Quaternion>();
                                        actionAnim.AddKeyFrame(KeyFrameChannel.OrientationW, boneName, time, ori.w);
                                        actionAnim.AddKeyFrame(KeyFrameChannel.OrientationX, boneName, time, ori.x);
                                        actionAnim.AddKeyFrame(KeyFrameChannel.OrientationY, boneName, time, ori.y);
                                        actionAnim.AddKeyFrame(KeyFrameChannel.OrientationZ, boneName, time, ori.z);
                                        break;
                                    case FileKeyFrameType.Translation:
                                        Vector3f pos = reader.Read<Vector3f>();
                                        actionAnim.AddKeyFrame(KeyFrameChannel.PositionX, boneName, time, pos.x);
                                        actionAnim.AddKeyFrame(KeyFrameChannel.PositionY, boneName, time, pos.y);
                                        actionAnim.AddKeyFrame(KeyFrameChannel.PositionZ, boneName, time, pos.z);
                                        break;
                                }
                            }
                        }
                    }
                    animations[actionName] = actionAnim;
                }
                return animations;
            }
        }

        static int CompareKeyFrames(KeyFrame a, KeyFrame b)
            => a.time.CompareTo(b.time);

        public SysCol.Dictionary<string, float> animationWeights = new SysCol.Dictionary<string, float>();

        internal SysCol.Dictionary<string, AdvancedLinkedList<KeyFrame>>[] frames
            = new SysCol.Dictionary<string, AdvancedLinkedList<KeyFrame>>[NUM_KEY_FRAME_CHANNELS];

        public string name { get; private set; }

        public Animation(string name)
        {
            this.name = name;
            for (int i = 0; i < frames.Length; i++)
                frames[i] = new SysCol.Dictionary<string, AdvancedLinkedList<KeyFrame>>();
        }

        public void AddKeyFrame(KeyFrameChannel channel, string bone, float time, float value)
        {
            // ignore meaningless values potentially loaded from 3rd party file formats
            if (float.IsNaN(value))
                return;

            AdvancedLinkedList<KeyFrame> lst;
            if (!frames[(byte)channel].TryGetValue(bone, out lst))
            {
                lst = frames[(byte)channel][bone] = new AdvancedLinkedList<KeyFrame>();
                animationWeights[bone] = 1;
            }

            System.Diagnostics.Debug.Assert(
                lst.All(f => f.time != time),
                $"Timestamp {time} already exists for {bone}.{channel} in rig {name}"
                );

            lst.AddSorted(new KeyFrame(time, value), CompareKeyFrames);
        }

        public void AddOrientationKeyFrame(string bone, float time, Quaternion value)
        {
            AddKeyFrame(KeyFrameChannel.OrientationW, bone, time, value.w);
            AddKeyFrame(KeyFrameChannel.OrientationX, bone, time, value.x);
            AddKeyFrame(KeyFrameChannel.OrientationY, bone, time, value.y);
            AddKeyFrame(KeyFrameChannel.OrientationZ, bone, time, value.z);
        }

        public void AddLocationKeyFrame(string bone, float time, Vector3f value)
        {
            AddKeyFrame(KeyFrameChannel.PositionX, bone, time, value.x);
            AddKeyFrame(KeyFrameChannel.PositionY, bone, time, value.y);
            AddKeyFrame(KeyFrameChannel.PositionZ, bone, time, value.z);
        }

        public void AddScalingKeyFrame(string bone, float time, Vector3f value)
        {
            AddKeyFrame(KeyFrameChannel.ScalingX, bone, time, value.x);
            AddKeyFrame(KeyFrameChannel.ScalingY, bone, time, value.y);
            AddKeyFrame(KeyFrameChannel.ScalingZ, bone, time, value.z);
        }

        public void RemoveKeyFrame(int channel, string bone, float time)
        {
            AdvancedLinkedList<KeyFrame> boneFrames = frames[channel][bone];
            if (boneFrames != null)
                foreach (KeyFrame frame in boneFrames)
                    if (frame.time == time)
                    {
                        boneFrames.RemoveCurrent();
                        return;
                    }
        }

        public void SetAnimationData(Rig rig, float progression, float totalWeight = 1)
        {
            if (totalWeight <= 0)
                return;

            foreach (SysCol.KeyValuePair<string, float> weight in animationWeights)
            {
                if (weight.Value <= 0)
                    continue;

                Rig.Bone bone = rig.root.GetBoneByName(weight.Key);
                if (bone != null)
                    bone.influences.Add(new Rig.Influence(GetTransform(bone.name, progression), weight.Value * totalWeight));
                else
                    System.Diagnostics.Debug.Fail($"Action {name} tries to animate bone \"{weight.Key}\" that does not exist in rig.");
            }
        }

        public void SetAnimationData(Rig.Bone bone, float progression, float weight = 1)
        {
            if (bone != null && weight > 0)
                bone.influences.Add(new Rig.Influence(GetTransform(bone.name, progression), animationWeights[bone.name] * weight));
        }

        float GetDefaultValue(KeyFrameChannel channel)
        {
            switch (channel)
            {
                case KeyFrameChannel.PositionX:
                case KeyFrameChannel.PositionY:
                case KeyFrameChannel.PositionZ:
                case KeyFrameChannel.OrientationX:
                case KeyFrameChannel.OrientationY:
                case KeyFrameChannel.OrientationZ:
                    return 0;

                case KeyFrameChannel.ScalingX:
                case KeyFrameChannel.ScalingY:
                case KeyFrameChannel.ScalingZ:
                case KeyFrameChannel.OrientationW:
                    return 1;

                default:
                    throw new System.InvalidOperationException($"{channel} is not a valid {nameof(KeyFrameChannel)}.");
            }
        }

        float EvaluateChannel(string bone, KeyFrameChannel channel, float progression)
        {
            AdvancedLinkedList<KeyFrame> boneFrames = null;
            if (!frames[(int)channel].TryGetValue(bone, out boneFrames) || boneFrames.length == 0)
                return GetDefaultValue(channel);

            if (boneFrames.length == 1)
                return boneFrames.first.value;

            KeyFrame prev = null, next = null;
            foreach (KeyFrame frame in boneFrames)
                if (frame.time >= progression)
                {
                    next = frame;
                    prev = boneFrames.GetCurrentForeachElement(-1);
                    if (prev == null)
                        prev = boneFrames.last;

                    break;
                }

            if (next == null)
            {
                prev = boneFrames.last;
                next = boneFrames.first;
            }

            float deltaKeyFrames = next.time - prev.time;
            if (deltaKeyFrames < 0)
                deltaKeyFrames++;

            float deltaRealTimeToPrevious = progression - prev.time;
            if (deltaRealTimeToPrevious < 0)
                deltaRealTimeToPrevious++;

            float f = deltaKeyFrames == 0 ? 1 : deltaRealTimeToPrevious / deltaKeyFrames;
            return prev.value + (next.value - prev.value) * f;
        }

        public virtual Matrix GetTransform(string bone, float progression)
        {
            Matrix transform = Matrix.IDENTITY;
            float[] interpolatedValues = new float[NUM_KEY_FRAME_CHANNELS];

            for (KeyFrameChannel type = 0; (byte)type < NUM_KEY_FRAME_CHANNELS; type++)
                interpolatedValues[(int)type] = EvaluateChannel(bone, type, progression);

            return Matrix.Scaling(
                    interpolatedValues[(int)KeyFrameChannel.ScalingX],
                    interpolatedValues[(int)KeyFrameChannel.ScalingY],
                    interpolatedValues[(int)KeyFrameChannel.ScalingZ]
                    )
                 * Matrix.RotationQuaternion(new Quaternion(
                    interpolatedValues[(int)KeyFrameChannel.OrientationX],
                    interpolatedValues[(int)KeyFrameChannel.OrientationY],
                    interpolatedValues[(int)KeyFrameChannel.OrientationZ],
                    interpolatedValues[(int)KeyFrameChannel.OrientationW]
                    ))
                 * Matrix.Translation(
                    interpolatedValues[(int)KeyFrameChannel.PositionX],
                    interpolatedValues[(int)KeyFrameChannel.PositionY],
                    interpolatedValues[(int)KeyFrameChannel.PositionZ]
                    );
        }
    }
}
