using System.Collections.Generic;
using Unity.Burst;
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;
using UnityEngine;

namespace Parallel_XPBD
{
    public class Helper
    {
        public static DistanceConstraint[] GetConstraintsToParticle(int particleId, XpbdMesh xpbdMesh)
        {
            List<DistanceConstraint> constraints = new List<DistanceConstraint>();
            for (int i = 0; i < 12; i++)
            {
                if (xpbdMesh.ParticleToConst[particleId * 12 + i]< 0) continue;
                
                constraints.Add(xpbdMesh.Distances[xpbdMesh.ParticleToConst[particleId * 12 + i]]);
            }//

            return constraints.ToArray();
        }

        public static Vector3[] FloatToVectorParallel(float3[] floats)
        {
            NativeArray<float3> floatsNative = new NativeArray<float3>(floats, Allocator.TempJob);
            NativeArray<Vector3> vecsNative = new NativeArray<Vector3>(floats.Length, Allocator.TempJob);

            VecTofloatArray job = new VecTofloatArray()
            {
                Floats = floatsNative,
                Vectors = vecsNative
            };
            job.Schedule(floatsNative.Length, 8).Complete();
            Vector3[] result = new Vector3[floats.Length];

            job.Vectors.CopyTo(result);
            floatsNative.Dispose();
            vecsNative.Dispose();
            return result;
        }

        [BurstCompile]
        private struct VecTofloatArray : IJobParallelFor
        {
            public NativeArray<float3> Floats;
            public NativeArray<Vector3> Vectors;

            public void Execute(int index)
            {
                Vectors[index] = Floats[index];
            }
        }
    }
}