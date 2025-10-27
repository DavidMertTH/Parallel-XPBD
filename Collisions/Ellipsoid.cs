using Unity.Mathematics;
using UnityEngine;

namespace myXpbd.Parallel_XPBD.Collisions
{
    public struct Ellipsoid
    {
        public float3 Position;
        public quaternion Rotation;
        public float3 HalfAxis;
    }
}