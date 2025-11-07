using System.Collections.Generic;
using System.Linq;
using Unity.Burst;
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;

namespace Parallel_XPBD
{
    public class ColorGroupXpbdSolver : ISoftBodySolver
    {
        public float[] Lambdas;
        private XpbdMesh _toSimulate;

        public ColorGroupXpbdSolver(XpbdMesh xpbdMesh)
        {
            _toSimulate = xpbdMesh;
            Lambdas = new float[_toSimulate.Distances.Length];
        }

        public void SolveConstraints(float timeStepLength, int subSteps, ref float3[] predictedPositions)
        {
            float subStepLength = timeStepLength / subSteps;
            //Lambdas = new float[_toSimulate.Distances.Length];

            NativeArray<Particle> nativeParticles = new NativeArray<Particle>(_toSimulate.Particles, Allocator.TempJob);
            NativeArray<DistanceConstraint> nativeDistances =
                new NativeArray<DistanceConstraint>(_toSimulate.Distances, Allocator.TempJob);
            
            NativeArray<float> nativeLambdasA = new NativeArray<float>(Lambdas, Allocator.TempJob);
            NativeArray<float> nativeLambdasB = new NativeArray<float>(Lambdas, Allocator.TempJob);

            NativeArray<float3> nativeBufferA = new NativeArray<float3>(predictedPositions, Allocator.TempJob);
            NativeArray<float3> nativeBufferB = new NativeArray<float3>(predictedPositions.Length, Allocator.TempJob);
            NativeArray<int> partToConst = new NativeArray<int>(_toSimulate.ParticleToConst, Allocator.TempJob);

            int numColors = 6;
            List<JobHandle> prevJobs = new List<JobHandle>();
            for (int s = 0; s < subSteps; s++)
            {
                for (int i = 0; i < numColors; i++)
                {
                    if (i % 2 == 0)
                    {
                        SolveConstraintGroupJob groupJob = new SolveConstraintGroupJob()
                        {
                            Distances = nativeDistances,
                            LambdasIn = nativeLambdasA,
                            LambdasOut = nativeLambdasB,
                            PredictedPositions = nativeBufferA,
                            ResultingPositions = nativeBufferB,
                            Particles = nativeParticles,
                            SubStepLength = subStepLength,
                            PartToConst = partToConst,
                            GroupToSolve = i
                        };
                        if (prevJobs.Count > 0)
                            prevJobs.Add(groupJob.Schedule(_toSimulate.Particles.Length, 8, prevJobs.Last()));
                        else prevJobs.Add(groupJob.Schedule(_toSimulate.Particles.Length, 8));
                    }
                    else
                    {
                        SolveConstraintGroupJob groupJob = new SolveConstraintGroupJob()
                        {
                            Distances = nativeDistances,
                            LambdasIn = nativeLambdasB,
                            LambdasOut = nativeLambdasA,
                            PredictedPositions = nativeBufferB,
                            ResultingPositions = nativeBufferA,
                            Particles = nativeParticles,
                            SubStepLength = subStepLength,
                            PartToConst = partToConst,
                            GroupToSolve = i
                        };
                        if (prevJobs.Count > 0)
                            prevJobs.Add(groupJob.Schedule(_toSimulate.Particles.Length, 8, prevJobs.Last()));
                        else prevJobs.Add(groupJob.Schedule(_toSimulate.Particles.Length, 8));
                    }
                    
                }
            }

            prevJobs.Last().Complete();
            nativeBufferA.CopyTo(predictedPositions);

            nativeParticles.Dispose();
            nativeDistances.Dispose();
            nativeLambdasA.Dispose();
            nativeLambdasB.Dispose();
            nativeBufferA.Dispose();
            nativeBufferB.Dispose();
            partToConst.Dispose();
        }
    }

    // [BurstCompile]
    public struct SolveConstraintGroupJob : IJobParallelFor
    {
        [ReadOnly] public NativeArray<Particle> Particles;
        [ReadOnly] public NativeArray<DistanceConstraint> Distances;
        [ReadOnly] public NativeArray<int> PartToConst;

        [ReadOnly] public NativeArray<float3> PredictedPositions;
        public NativeArray<float3> ResultingPositions;

        [ReadOnly] public NativeArray<float> LambdasIn;
        public NativeArray<float> LambdasOut;

        public float SubStepLength;
        public int GroupToSolve;


        public void Execute(int index)
        {
            if (Particles[index].InvMass == 0)
            {
                ResultingPositions[index] = PredictedPositions[index]; //+ moveDirSum * 0.1f;
                return;
            }

            float3 moveDirSum = new float3();

            for (int i = 0; i < 12; i++)
            {
                float3 posSelf = float3.zero;
                float3 posOther = float3.zero;
                int constrainIndex = PartToConst[index * 12 + i];
                if (constrainIndex == -1) continue;
                var constraint = Distances[constrainIndex];
                Particle selfParticle = new Particle();
                Particle otherParticle = new Particle();

                if (constraint.ParticleA == index)
                {
                    if (constraint.PartAGroup != GroupToSolve) continue;

                    selfParticle = Particles[constraint.ParticleA];
                    otherParticle = Particles[constraint.ParticleB];

                    posSelf = PredictedPositions[constraint.ParticleA];
                    posOther = PredictedPositions[constraint.ParticleB];
                }

                if (constraint.ParticleB == index)
                {
                    if (constraint.PartBGroup != GroupToSolve) continue;

                    selfParticle = Particles[constraint.ParticleB];
                    otherParticle = Particles[constraint.ParticleA];

                    posSelf = PredictedPositions[constraint.ParticleB];
                    posOther = PredictedPositions[constraint.ParticleA];
                }

                float3 gradient = posOther - posSelf;
                float distError = math.length(gradient) - constraint.RestLenght;

                gradient = math.normalize(gradient);

                float alpha = constraint.Compliance / (SubStepLength * SubStepLength);

                float invMassOne = selfParticle.InvMass;
                float invMassTwo = otherParticle.InvMass;
                float massSum = invMassOne + invMassTwo;


                float denom = massSum + alpha;
                float dlambda = (-distError - alpha * LambdasIn[constrainIndex]) / denom;

                moveDirSum -= gradient * (invMassOne * dlambda);

                LambdasOut[constrainIndex] += dlambda / 2;
            }

            ResultingPositions[index] = PredictedPositions[index] + moveDirSum;
        }
    }
}