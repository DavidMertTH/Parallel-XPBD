using System.Numerics;
using Unity.Burst;
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;

namespace Parallel_XPBD
{
    public class JacobiXpbdSolver : ISoftBodySolver
    {
        public float3[] Gradients;
        public float[] Lambdas;
        private XpbdMesh _toSimulate;

        public JacobiXpbdSolver(XpbdMesh xpbdMesh)
        {
            _toSimulate = xpbdMesh;
            Gradients = new float3[_toSimulate.Distances.Length];
            Lambdas = new float[_toSimulate.Distances.Length];
        }

        public void SolveDistanceConstraints(float timeStepLength, int subSteps, ref float3[] predictedPositions)
        {
            float subStepLength = timeStepLength / subSteps;
            NativeArray<Particle> nativeParticles = new NativeArray<Particle>(_toSimulate.Particles, Allocator.TempJob);
            NativeArray<DistanceConstraint> nativeDistances =
                new NativeArray<DistanceConstraint>(_toSimulate.Distances, Allocator.TempJob);
            NativeArray<float> nativeLambdas = new NativeArray<float>(Lambdas, Allocator.TempJob);
            NativeArray<float3> nativePredictedPositions =
                new NativeArray<float3>(predictedPositions, Allocator.TempJob);

            NativeArray<float3> deltaResultOne =
                new NativeArray<float3>(nativeDistances.Length, Allocator.TempJob);
            NativeArray<float3> deltaResultTwo =
                new NativeArray<float3>(nativeDistances.Length, Allocator.TempJob);

            NativeArray<int> partToConst = new NativeArray<int>(_toSimulate.ParticleToConst, Allocator.TempJob);
            NativeArray<float> erorrs = new NativeArray<float>(_toSimulate.Distances.Length, Allocator.TempJob);

            JobHandle[] jobHandleConst = new JobHandle[subSteps];
            JobHandle[] jobHandleJacobi = new JobHandle[subSteps];

            for (int i = 0; i < subSteps; i++)
            {
                SolveConstraintsJob job = new SolveConstraintsJob()
                {
                    Particles = nativeParticles,
                    Distances = nativeDistances,
                    Lambdas = nativeLambdas,
                    PredictedPositions = nativePredictedPositions,
                    SubStepLength = subStepLength,
                    DeltaResOne = deltaResultOne,
                    DeltaResTwo = deltaResultTwo,
                };
                if (i == 0)
                {
                    jobHandleConst[i] = job.Schedule(nativeDistances.Length, 32);
                }
                else
                {
                    jobHandleConst[i] = job.Schedule(nativeDistances.Length, 32, jobHandleJacobi[i - 1]);
                }

                AddJacobiJob jacobiJobJob = new AddJacobiJob()
                {
                    Distances = nativeDistances,
                    ResOne = deltaResultOne,
                    ResTwo = deltaResultTwo,
                    PredictedPositions = nativePredictedPositions,
                    PartToConst = partToConst
                };

                jobHandleJacobi[i] = jacobiJobJob.Schedule(nativeParticles.Length, 32, jobHandleConst[i]);

                // job.PredictedPositions.CopyFrom(nativePredictedPositions);
            }

            jobHandleJacobi[subSteps - 1].Complete();
            GetErrorJob errorJob = new GetErrorJob()
            {
                Particles = nativeParticles,
                Distances = nativeDistances,
                PredictedPositions = nativePredictedPositions,
                Errors = erorrs,
            };
            errorJob.Schedule(nativeDistances.Length, 32).Complete();

            nativePredictedPositions.CopyTo(predictedPositions);

            nativeParticles.Dispose();
            nativeDistances.Dispose();
            nativeLambdas.Dispose();
            nativePredictedPositions.Dispose();
            deltaResultOne.Dispose();
            deltaResultTwo.Dispose();
            partToConst.Dispose();
            erorrs.Dispose();
        }
    }
    
    [BurstCompile]
    public struct SolveConstraintsJob : IJobParallelFor
    {
        [ReadOnly] public NativeArray<Particle> Particles;
        [ReadOnly] public NativeArray<DistanceConstraint> Distances;
        [ReadOnly] public NativeArray<float3> PredictedPositions;

        public NativeArray<float3> DeltaResOne;
        public NativeArray<float3> DeltaResTwo;
        public NativeArray<float> Lambdas;
        
        public float SubStepLength;

        public void Execute(int index)
        {
            var constraint = Distances[index];
            float3 gradient = PredictedPositions[constraint.ParticleA] - PredictedPositions[constraint.ParticleB];
            float distError = math.length(gradient) - constraint.RestLenght;
            gradient = math.normalize(gradient);

            float alpha = constraint.Compliance / (SubStepLength * SubStepLength);

            float invMassOne = Particles[constraint.ParticleA].InvMass;
            float invMassTwo = Particles[constraint.ParticleB].InvMass;
            float massSum = invMassOne + invMassTwo;
            
            float denom = massSum + alpha;
            float dlambda = (-distError - alpha * Lambdas[index]) / denom;

            DeltaResOne[index] = gradient * (invMassOne * dlambda);
            DeltaResTwo[index] = -gradient * (invMassTwo * dlambda);

            Lambdas[index] += dlambda;
        }
    }
    [BurstCompile]
    public struct GetErrorJob : IJobParallelFor
    {
        [ReadOnly] public NativeArray<Particle> Particles;
        [ReadOnly] public NativeArray<DistanceConstraint> Distances;
        [ReadOnly] public NativeArray<float3> PredictedPositions;
        public NativeArray<float> Errors;

        public void Execute(int index)
        {
            DistanceConstraint constraint = Distances[index];
            float3 grad = PredictedPositions[constraint.ParticleA] - PredictedPositions[constraint.ParticleB];
            Errors[index] = math.length(grad) - constraint.RestLenght;
        }
    }

    [BurstCompile]
    public struct AddJacobiJob : IJobParallelFor
    {
        [ReadOnly] public NativeArray<DistanceConstraint> Distances;
        [ReadOnly] public NativeArray<float3> ResOne;
        [ReadOnly] public NativeArray<float3> ResTwo;
        [ReadOnly] public NativeArray<int> PartToConst;

        public NativeArray<float3> PredictedPositions;

        public void Execute(int index)
        {
            for (int i = 0; i < 12; i++)
            {
                int constrainIndex = PartToConst[index * 12 + i];
                if (constrainIndex == -1) continue;
                DistanceConstraint constrain = Distances[constrainIndex];
                if (constrain.ParticleA == index)
                {
                    PredictedPositions[index] += ResOne[constrainIndex] * 0.3f;
                }

                if (constrain.ParticleB == index)
                {
                    PredictedPositions[index] += ResTwo[constrainIndex] * 0.3f;
                }
            }
        }
    }
}