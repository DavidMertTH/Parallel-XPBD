using System.Collections.Generic;
using System.Linq;
using System.Numerics;
using Unity.Burst;
using Unity.Collections;
using Unity.Jobs;
using Unity.Jobs.LowLevel.Unsafe;
using Unity.Mathematics;
using Unity.VisualScripting;

namespace Parallel_XPBD
{
    public class JacobiXpbdSolver : ISoftBodySolver
    {
        public float3[] Gradients;
        public float[] Lambdas;
        private XpbdMesh _toSimulate;


        private NativeArray<Particle> _nativeParticles;
        private NativeArray<DistanceConstraint> _nativeDistances;
        private NativeArray<float> _nativeLambdas;
        private NativeArray<float> _erorrs;
        private NativeArray<float3> _nativePredictedPositions;
        private NativeArray<float3> _deltaResultOne;
        private NativeArray<float3> _deltaResultTwo;
        private NativeArray<int> _partToConst;

        private JobHandle _handle;
        private bool _isRunning;

        public JacobiXpbdSolver(XpbdMesh xpbdMesh)
        {
            _toSimulate = xpbdMesh;
            Gradients = new float3[_toSimulate.Distances.Length];
            Lambdas = new float[_toSimulate.Distances.Length];
            _toSimulate.Destroyed += OnDestroy;
        }

        [BurstCompile]
        public void SolveDistanceConstraints(float timeStepLength, int subSteps, ref float3[] predictedPositions,
            bool solveCollisions)
        {
            int collisionStepsPerSubsteps = 5;
            _toSimulate.xpbd.TimeLogger.StartSimClockwatch();
            float subStepLength = timeStepLength / subSteps;
            _nativeParticles = new NativeArray<Particle>(_toSimulate.Particles, Allocator.TempJob);
            _nativeDistances = new NativeArray<DistanceConstraint>(_toSimulate.Distances, Allocator.TempJob);
            _nativeLambdas = new NativeArray<float>(Lambdas, Allocator.TempJob);
            _nativePredictedPositions = new NativeArray<float3>(predictedPositions, Allocator.TempJob);

            _deltaResultOne =
                new NativeArray<float3>(_nativeDistances.Length, Allocator.TempJob);
            _deltaResultTwo =
                new NativeArray<float3>(_nativeDistances.Length, Allocator.TempJob);

            _partToConst = new NativeArray<int>(_toSimulate.ParticleToConst, Allocator.TempJob);
            _erorrs = new NativeArray<float>(_toSimulate.Distances.Length, Allocator.TempJob);

            List<JobHandle> prevJobs = new List<JobHandle>();

            for (int i = 0; i < subSteps; i++)
            {
                SolveConstraintsJob job = new SolveConstraintsJob()
                {
                    Particles = _nativeParticles,
                    Distances = _nativeDistances,
                    Lambdas = _nativeLambdas,
                    PredictedPositions = _nativePredictedPositions,
                    SubStepLength = subStepLength,
                    DeltaResOne = _deltaResultOne,
                    DeltaResTwo = _deltaResultTwo,
                };
                if (i == 0) prevJobs.Add(job.Schedule(_nativeDistances.Length, 32));
                else prevJobs.Add(job.Schedule(_nativeDistances.Length, 32, prevJobs.Last()));


                AddJacobiJob jacobiJobJob = new AddJacobiJob()
                {
                    Distances = _nativeDistances,
                    ResOne = _deltaResultOne,
                    ResTwo = _deltaResultTwo,
                    PredictedPositions = _nativePredictedPositions,
                    PartToConst = _partToConst
                };
                prevJobs.Add(jacobiJobJob.Schedule(_nativeParticles.Length, 32, prevJobs.Last()));
                if (solveCollisions)
                {
                    if (i % collisionStepsPerSubsteps == 0)
                    {
                        prevJobs.Add(
                            _toSimulate.xpbd.HashMapEllipsoids.AccessHashMapParallel(_nativePredictedPositions,
                                prevJobs.Last()));
                    }
                }
            }

            _isRunning = true;
            _handle = prevJobs.Last();
        }

        public void FinnishJob(ref float3[] predictedPositions)
        {
            if (!_isRunning) return;
            bool isDone = _handle.IsCompleted;

            _handle.Complete();
            if (predictedPositions != null) _nativePredictedPositions.CopyTo(predictedPositions);
            _toSimulate.xpbd.TimeLogger.StopSimClockwatch(!isDone);

            _toSimulate.xpbd.SpatialHashMap.DisposeHashMap();
            _nativeParticles.Dispose();
            _nativeDistances.Dispose();
            _nativeLambdas.Dispose();
            _nativePredictedPositions.Dispose();
            _deltaResultOne.Dispose();
            _deltaResultTwo.Dispose();
            _partToConst.Dispose();
            _erorrs.Dispose();
        }

        public void OnDestroy()
        {
            FinnishJob(ref _toSimulate.xpbd.ParticlePositions);
        }

        public void SolveDistanceConstraints(float timeStepLength, int subSteps, ref float3[] predictedPositions)
        {
            SolveDistanceConstraints(timeStepLength, subSteps, ref predictedPositions, _toSimulate.handleCollisions);
            FinnishJob(ref predictedPositions);
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