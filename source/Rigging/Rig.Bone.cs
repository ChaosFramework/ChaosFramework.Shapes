using ChaosFramework.Collections;
using ChaosFramework.Math;
using ChaosFramework.Math.Vectors;
using SysCol = System.Collections.Generic;

namespace ChaosFramework.Shapes.Rigging
{
    partial class Rig
    {
        public class Bone
        {
            static readonly Matrix OUT_OF_BLENDER_SPACE = new Matrix(
                1, 0, 0, 0,
                0, 0, 1, 0,
                0, 1, 0, 0,
                0, 0, 0, 1
            );

            public string name;
            public Bone parent;
            public LinkedList<Bone> children = new LinkedList<Bone>();
            public Matrix restTransform = Matrix.IDENTITY;
            public float length;
            public LinkedList<Influence> influences = new LinkedList<Influence>();

            public Matrix transform { get; private set; } = Matrix.IDENTITY;

            public Bone(string name)
            {
                this.name = name;
            }

            public Vector3f GetPosition()
                => GetBoneTransform().row3.xyz;

            public Vector3f GetDirection()
                => GetBoneTransform().row2.xyz;

            public Matrix GetBoneTransform()
                => OUT_OF_BLENDER_SPACE
                 * restTransform
                 * transform;

            public Bone GetBoneByName(string name)
            {
                if (name == this.name)
                    return this;

                foreach (Bone b in children)
                {
                    Bone result = b.GetBoneByName(name);
                    if (result != null)
                        return result;
                }

                return null;
            }

            internal void ClearInfluences()
            {
                foreach (Bone child in children)
                    child.ClearInfluences();

                influences.Clear();
                transform = Matrix.IDENTITY;
            }

            public void ComputeTransforms()
            {
                if (parent != null)
                {
                    if (influences.length == 0)
                        transform = parent.transform;
                    else
                    {
                        Matrix trans = new Matrix();
                        float weightSum = 0;
                        foreach (Influence inf in influences)
                        {
                            trans += inf.transform * inf.weight;
                            weightSum += inf.weight;
                        }
                        trans = trans / weightSum;

                        transform = Matrix.Invert(restTransform) * trans * restTransform * parent.transform;
                    }
                }
                else
                    transform = Matrix.IDENTITY;

                foreach (Bone child in children)
                    child.ComputeTransforms();
            }

            public Bone Clone(Bone parent)
            {
                Bone b = new Bone(name);
                b.parent = parent;
                b.length = length;
                b.restTransform = restTransform;
                foreach (Bone child in children)
                    b.children.Add(child.Clone(b));

                return b;
            }

            public SysCol.IEnumerable<Bone> EnumerateBones()
            {
                yield return this;
                foreach (Bone child in children)
                    foreach (Bone childBone in child.EnumerateBones())
                        yield return childBone;
            }
        }
    }
}
