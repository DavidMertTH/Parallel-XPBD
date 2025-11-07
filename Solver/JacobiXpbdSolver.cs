using System.Collections.Generic;
using System.Linq;
using System.Numerics;
using Unity.Burst;
using Unity.Collections;
using Unity.Jobs;
using Unity.Jobs.LowLevel.Unsafe;
using Unity.Mathematics;
using Unity.VisualScripting;
using UnityEngine;

namespace Parallel_XPBD
{
    public class JacobiXpbdSolver : ISoftBodySolver
    {
        public float3[] Gradients;
        private float[] _lambdas;
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
            _lambdas = new float[_toSimulate.Distances.Length];
            _toSimulate.Destroyed += OnDestroy;
        }

        public void ScheduleConstraintChain(float timeStepLength, int subSteps, ref float3[] predictedPositions,
            bool solveCollisions)
        {
            int collisionStepsPerSubsteps = 1 + (int)((float)subSteps * (1f - _toSimulate.collisionResolution));
            _toSimulate.xpbd.TimeLogger.StartSimClockwatch();
            float subStepLength = timeStepLength / subSteps;
            List<JobHandle> prevJobs = new List<JobHandle>();
            AllocateNativeArrays(ref predictedPositions);

            for (int i = 0; i < subSteps; i++)
            {
                AttachDistanceConstraint(ref prevJobs, subStepLength);

                if (solveCollisions && i % collisionStepsPerSubsteps == 0)
                {
                    AttachCollisionConstraints(ref prevJobs, i, subSteps);
                }
            }


            _handle = prevJobs.Last();
            _isRunning = true;
        }

        private void AllocateNativeArrays(ref float3[] predictedPositions)
        {
            _nativeParticles = new NativeArray<Particle>(_toSimulate.Particles, Allocator.TempJob);
            _nativeDistances = new NativeArray<DistanceConstraint>(_toSimulate.Distances, Allocator.TempJob);
            _nativeLambdas = new NativeArray<float>(_lambdas, Allocator.TempJob);
            _nativePredictedPositions = new NativeArray<float3>(predictedPositions, Allocator.TempJob);

            _deltaResultOne =
                new NativeArray<float3>(_nativeDistances.Length, Allocator.TempJob);
            _deltaResultTwo =
                new NativeArray<float3>(_nativeDistances.Length, Allocator.TempJob);

            _partToConst = new NativeArray<int>(_toSimulate.ParticleToConst, Allocator.TempJob);
        }

        public void AttachDistanceConstraint(ref List<JobHandle> constraintChain, float subStepLength)
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
            if (constraintChain.Count == 0) constraintChain.Add(job.Schedule(_nativeDistances.Length, 32));
            else constraintChain.Add(job.Schedule(_nativeDistances.Length, 32, constraintChain.Last()));

            AddJacobiJob jacobiJobJob = new AddJacobiJob()
            {
                Distances = _nativeDistances,
                ResOne = _deltaResultOne,
                ResTwo = _deltaResultTwo,
                PredictedPositions = _nativePredictedPositions,
                PartToConst = _partToConst
            };
            constraintChain.Add(jacobiJobJob.Schedule(_nativeParticles.Length, 32, constraintChain.Last()));
        }

        public void AttachCollisionConstraints(ref List<JobHandle> previousJobs, int currentSubStepIndex,
            int maxSubsteps)
        {
            previousJobs.Add(
                _toSimulate.xpbd.HashMapEllipsoids.AccessHashMapParallel(_nativePredictedPositions,
                    previousJobs.Last(), currentSubStepIndex, maxSubsteps));
        }

        public void FinnishJob(ref float3[] predictedPositions)
        {
            if (!_isRunning) return;
            bool isDone = _handle.IsCompleted;

            _handle.Complete();
            if (_nativePredictedPositions.IsCreated) _nativePredictedPositions.CopyTo(predictedPositions);
            _toSimulate.xpbd.TimeLogger.StopSimClockwatch(!isDone);

            _toSimulate.xpbd.HashMapEllipsoids.DisposeHashMap();
            _toSimulate.xpbd.SphereMap.DisposeHashMap();

            if (_nativeParticles.IsCreated) _nativeParticles.Dispose();
            if (_nativeDistances.IsCreated) _nativeDistances.Dispose();
            if (_nativeLambdas.IsCreated) _nativeLambdas.Dispose();
            if (_nativePredictedPositions.IsCreated) _nativePredictedPositions.Dispose();
            if (_deltaResultOne.IsCreated) _deltaResultOne.Dispose();
            if (_deltaResultTwo.IsCreated) _deltaResultTwo.Dispose();
            if (_partToConst.IsCreated) _partToConst.Dispose();
            if (_erorrs.IsCreated) _erorrs.Dispose();
        }

        public void OnDestroy()
        {
            FinnishJob(ref _toSimulate.xpbd.ParticlePositions);
        }

        public void SolveConstraints(float timeStepLength, int subSteps, ref float3[] predictedPositions)
        {
            ScheduleConstraintChain(timeStepLength, subSteps, ref predictedPositions, _toSimulate.handleCollisions);
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
            if (!Particles[constraint.ParticleA].IsActive || !Particles[constraint.ParticleB].IsActive) return;
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