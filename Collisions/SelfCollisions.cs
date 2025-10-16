using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;

namespace myXpbd.Parallel_XPBD.Collisions
{
    public class SelfCollisions
    {
        public float3[] HandleSelfCollisions(float3[] positions)
        {
            SelfCollisionJob job = new SelfCollisionJob()
            {
                InputPositions = new NativeArray<float3>(positions, Allocator.TempJob),
                ResultingPositions = new NativeArray<float3>(positions, Allocator.TempJob),
            };
            job.Schedule(positions.Length, 32).Complete();
            return job.ResultingPositions.ToArray();
        }

        public struct SelfCollisionJob : IJobParallelFor
        {
            [ReadOnly] public NativeArray<float3> InputPositions;
            public NativeArray<float3> ResultingPositions;

            public void Execute(int index)
            {
                float3 selfPosition = InputPositions[index];
                for (int i = 0; i < InputPositions.Length; i++)
                {
                    if (i == index) continue;
                    float distance = math.length(selfPosition - InputPositions[i]);
                    if (distance > 0.6f) continue;
                    ResultingPositions[index] += (selfPosition - InputPositions[i]);
                }
            }
        }
    }
}