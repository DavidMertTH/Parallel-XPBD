using Parallel_XPBD;
using Unity.Burst;
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;
using Unity.VisualScripting;
using UnityEditor.PackageManager;
using UnityEngine;
using UnityEngine.UIElements;

public class Xpbd
{
    private XpbdMesh _toSimulate;
    public float[] DistError;
    public Vector3[] Gradients;
    public float TimeStepLength = 1f / 200;
    public Vector3[] PredictedPositions;
    public Vector3[] PredictedVelocitys;
    public float[] Lambdas;
    public float Dampening;
    public float Compliance;
    public Solver SolverToUse;

    public Vector3[] CurrentParticlePositions;

    public enum Solver
    {
        Seriel,
        Jacobi,
        ParticleColorGroups
    }

    public Xpbd(XpbdMesh toSimulate)
    {
        _toSimulate = toSimulate;
        DistError = new float[_toSimulate.Distances.Length];
        Gradients = new Vector3[_toSimulate.Distances.Length];
        PredictedPositions = new Vector3[_toSimulate.Particles.Length];
        PredictedVelocitys = new Vector3[_toSimulate.Particles.Length];
    }

    public void Simulate(int subSteps)
    {
        Lambdas = new float[_toSimulate.Distances.Length];
        Integrate();
        if (SolverToUse == Solver.Seriel)
        {
            SolveConstraints(subSteps);
        }

        if (SolverToUse == Solver.Jacobi)
        {
            SolveConstraintsParallel(subSteps);
        }

        if (SolverToUse == Solver.ParticleColorGroups)
        {
            SolveConstGroups(subSteps);
        }

        UpdatePartícles();
    }

    private void Integrate()
    {
        NativeArray<Particle> nativeParticles = new NativeArray<Particle>(_toSimulate.Particles, Allocator.TempJob);
        NativeArray<Vector3> nativePredictedPositions = new NativeArray<Vector3>(PredictedPositions, Allocator.TempJob);

        IntegrateJob integrationJob = new IntegrateJob()
        {
            Dampening = Dampening,
            TimeStepLength = TimeStepLength,
            PredictedPositions = nativePredictedPositions,
            Particles = nativeParticles
        };
        integrationJob.Schedule(_toSimulate.Particles.Length, 32).Complete();
        nativePredictedPositions.CopyTo(PredictedPositions);
        nativeParticles.Dispose();
        nativePredictedPositions.Dispose();
    }

    private void SolveConstraints(int subSteps)
    {
        float subStepLength = TimeStepLength / subSteps;
        for (int s = 0; s < subSteps; s++)
        {
            DistanceConstraint constraint;

            for (int i = 0; i < _toSimulate.Distances.Length; i++)
            {
                constraint = _toSimulate.Distances[i];
                Gradients[i] = PredictedPositions[constraint.ParticleA] - PredictedPositions[constraint.ParticleB];
                DistError[i] = Gradients[i].magnitude - constraint.RestLenght;
                Gradients[i].Normalize();
                // Gradients[i] *= DistError[i];

                float alpha = constraint.Compliance / (subStepLength * subStepLength);

                float invMassOne = _toSimulate.Particles[constraint.ParticleA].InvMass;
                float invMassTwo = _toSimulate.Particles[constraint.ParticleB].InvMass;
                float massSum = invMassOne + invMassTwo;


                float denom = massSum + alpha;
                float dlambda = (-DistError[i] - alpha * Lambdas[i]) / denom;

                if (invMassOne > 0f)
                    PredictedPositions[constraint.ParticleA] += Gradients[i] * (invMassOne * dlambda);

                if (invMassTwo > 0f)
                    PredictedPositions[constraint.ParticleB] += -Gradients[i] * (invMassTwo * dlambda);

                Lambdas[i] += dlambda;
            }
        }
    }

    private void SolveConstGroups(int subSteps)
    {
        float subStepLength = TimeStepLength / subSteps;
        //Lambdas = new float[_toSimulate.Distances.Length];

        NativeArray<Particle> nativeParticles = new NativeArray<Particle>(_toSimulate.Particles, Allocator.TempJob);
        NativeArray<DistanceConstraint> nativeDistances =
            new NativeArray<DistanceConstraint>(_toSimulate.Distances, Allocator.TempJob);
        NativeArray<float> nativeLambdas = new NativeArray<float>(Lambdas, Allocator.TempJob);
        NativeArray<Vector3> nativePredictedPositions = new NativeArray<Vector3>(PredictedPositions, Allocator.TempJob);
        NativeArray<Vector3> nativeResultingPositions =
            new NativeArray<Vector3>(PredictedPositions.Length, Allocator.TempJob);
        NativeArray<int> partToConst = new NativeArray<int>(_toSimulate.ParticleToConst, Allocator.TempJob);
        NativeArray<float> erorrs = new NativeArray<float>(DistError, Allocator.TempJob);


        JobHandle[] prevJobs = new JobHandle[subSteps];
        for (int s = 0; s < subSteps; s++)
        {
            SolveConstraintGroupJob groupJob = new SolveConstraintGroupJob()
            {
                Distances = nativeDistances,
                Lambdas = nativeLambdas,
                PredictedPositions = nativePredictedPositions,
                ResultingPositions = nativeResultingPositions,
                Particles = nativeParticles,
                SubStepLength = subStepLength,
                PartToConst = partToConst,
                GroupToSolve = 0
            };
            JobHandle handle = new JobHandle();
            if (s == 0)
            {
                handle = groupJob.Schedule(_toSimulate.Particles.Length, 8);
            }
            else
            {
                handle = groupJob.Schedule(_toSimulate.Particles.Length, 8, prevJobs[s - 1]);
            }

            // nativePredictedPositions.CopyFrom(nativeResultingPositions);
            SolveConstraintGroupJob groupJob1 = new SolveConstraintGroupJob()
            {
                Distances = nativeDistances,
                Lambdas = nativeLambdas,
                PredictedPositions = nativeResultingPositions,
                ResultingPositions = nativePredictedPositions,
                Particles = nativeParticles,
                SubStepLength = subStepLength,
                PartToConst = partToConst,
                GroupToSolve = 1
            };
            JobHandle handle1 = groupJob1.Schedule(_toSimulate.Particles.Length, 8, handle);
            SolveConstraintGroupJob groupJob2 = new SolveConstraintGroupJob()
            {
                Distances = nativeDistances,
                Lambdas = nativeLambdas,
                PredictedPositions = nativePredictedPositions,
                ResultingPositions = nativeResultingPositions,
                Particles = nativeParticles,
                SubStepLength = subStepLength,
                PartToConst = partToConst,
                GroupToSolve = 2
            };
            JobHandle handle2 = groupJob2.Schedule(_toSimulate.Particles.Length, 8, handle1);
            SolveConstraintGroupJob groupJob3 = new SolveConstraintGroupJob()
            {
                Distances = nativeDistances,
                Lambdas = nativeLambdas,
                PredictedPositions = nativeResultingPositions,
                ResultingPositions = nativePredictedPositions,
                Particles = nativeParticles,
                SubStepLength = subStepLength,
                PartToConst = partToConst,
                GroupToSolve = 3
            };
            JobHandle handle3 = groupJob3.Schedule(_toSimulate.Particles.Length, 8, handle2);
            SolveConstraintGroupJob groupJob4 = new SolveConstraintGroupJob()
            {
                Distances = nativeDistances,
                Lambdas = nativeLambdas,
                PredictedPositions = nativePredictedPositions,
                ResultingPositions = nativeResultingPositions,
                Particles = nativeParticles,
                SubStepLength = subStepLength,
                PartToConst = partToConst,
                GroupToSolve = 4
            };
            JobHandle handle4 = groupJob4.Schedule(_toSimulate.Particles.Length, 8, handle3);
            SolveConstraintGroupJob groupJob5 = new SolveConstraintGroupJob()
            {
                Distances = nativeDistances,
                Lambdas = nativeLambdas,
                PredictedPositions = nativeResultingPositions,
                ResultingPositions = nativePredictedPositions,
                Particles = nativeParticles,
                SubStepLength = subStepLength,
                PartToConst = partToConst,
                GroupToSolve = 5
            };
            if (s == subSteps - 1)
            {
                groupJob5.Schedule(_toSimulate.Particles.Length, 8, handle4).Complete();
            }
            else
            {
                prevJobs[s] = groupJob5.Schedule(_toSimulate.Particles.Length, 8, handle4);
            }
        }

        erorrs.CopyTo(DistError);
        nativePredictedPositions.CopyTo(PredictedPositions);

        nativeParticles.Dispose();
        nativeDistances.Dispose();
        nativeLambdas.Dispose();
        nativePredictedPositions.Dispose();
        nativeResultingPositions.Dispose();
        partToConst.Dispose();
        erorrs.Dispose();
    }

    private void UpdatePartícles()
    {
        NativeArray<Particle> nativeParticles = new NativeArray<Particle>(_toSimulate.Particles, Allocator.TempJob);
        NativeArray<Vector3> nativePredictedPositions =
            new NativeArray<Vector3>(PredictedPositions, Allocator.TempJob);

        PositionUpdateJob updateJob = new PositionUpdateJob()
        {
            Particles = nativeParticles,
            PredictedPositions = nativePredictedPositions,
            TimeStepLength = TimeStepLength,
        };
        updateJob.Schedule(_toSimulate.Particles.Length, 32).Complete();

        if (CurrentParticlePositions == null) CurrentParticlePositions = new Vector3[_toSimulate.Particles.Length];

        nativePredictedPositions.CopyTo(CurrentParticlePositions);
        nativeParticles.CopyTo(_toSimulate.Particles);
        nativeParticles.Dispose();
        nativePredictedPositions.Dispose();
    }


    private void SolveConstraintsParallel(int subSteps)
    {
        float subStepLength = TimeStepLength / subSteps;
        NativeArray<Particle> nativeParticles = new NativeArray<Particle>(_toSimulate.Particles, Allocator.TempJob);
        NativeArray<DistanceConstraint> nativeDistances =
            new NativeArray<DistanceConstraint>(_toSimulate.Distances, Allocator.TempJob);
        NativeArray<float> nativeLambdas = new NativeArray<float>(Lambdas, Allocator.TempJob);
        NativeArray<Vector3> nativePredictedPositions =
            new NativeArray<Vector3>(PredictedPositions, Allocator.TempJob);

        NativeArray<Vector3> deltaResultOne =
            new NativeArray<Vector3>(nativeDistances.Length, Allocator.TempJob);
        NativeArray<Vector3> deltaResultTwo =
            new NativeArray<Vector3>(nativeDistances.Length, Allocator.TempJob);

        NativeArray<int> partToConst = new NativeArray<int>(_toSimulate.ParticleToConst, Allocator.TempJob);
        NativeArray<float> erorrs = new NativeArray<float>(DistError, Allocator.TempJob);

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

            AddJacobi JacobiJob = new AddJacobi()
            {
                Distances = nativeDistances,
                ResOne = deltaResultOne,
                ResTwo = deltaResultTwo,
                PredictedPositions = nativePredictedPositions,
                PartToConst = partToConst
            };

            jobHandleJacobi[i] = JacobiJob.Schedule(nativeParticles.Length, 32, jobHandleConst[i]);

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

        erorrs.CopyTo(DistError);
        nativePredictedPositions.CopyTo(PredictedPositions);

        nativeParticles.Dispose();
        nativeDistances.Dispose();
        nativeLambdas.Dispose();
        nativePredictedPositions.Dispose();
        deltaResultOne.Dispose();
        deltaResultTwo.Dispose();
        partToConst.Dispose();
        erorrs.Dispose();
    }

    public void HandleCollisions(SpatialHashMap hashMap)
    {
        _toSimulate.transform.TransformPoints(CurrentParticlePositions);
        Vector3[] displacement = hashMap.AccessHashMapParallel(CurrentParticlePositions);
        _toSimulate.transform.InverseTransformPoints(CurrentParticlePositions);


        for (int i = 0; i < _toSimulate.Particles.Length; i++)
        {
            if (displacement[i] == Vector3.zero) continue;
            Vector3 newPosition = _toSimulate.Particles[i].Position + displacement[i] + displacement[i].normalized * 0.01f;

            Particle particle = new Particle()
            {
                Position = newPosition,
                Velocity = newPosition - _toSimulate.Particles[i].Position,
                InvMass = _toSimulate.Particles[i].InvMass,
            };
            _toSimulate.Particles[i] = particle;
        }
    }

    [BurstCompile]
    public struct HandleCollisionsJob : IJobParallelFor
    {
        public NativeArray<Vector3> Offsets;
        public NativeArray<Vector3> Positions;

        public void Execute(int index)
        {
            Positions[index] += Offsets[index];
        }
    }


    [BurstCompile]
    private struct PositionUpdateJob : IJobParallelFor
    {
        public NativeArray<Particle> Particles;
        [ReadOnly] public NativeArray<Vector3> PredictedPositions;
        public float TimeStepLength;

        public void Execute(int index)
        {
            if (Particles[index].InvMass == 0) return;
            Particle particle = new Particle()
            {
                InvMass = Particles[index].InvMass,
                Position = PredictedPositions[index],
                Velocity = (PredictedPositions[index] - Particles[index].Position) / TimeStepLength,
            };
            Particles[index] = particle;
        }
    }

    [BurstCompile]
    private struct IntegrateJob : IJobParallelFor
    {
        [ReadOnly] public NativeArray<Particle> Particles;
        public NativeArray<Vector3> PredictedPositions;
        public float Dampening;
        public float TimeStepLength;

        public void Execute(int index)
        {
            Vector3 gravity = new Vector3(0, -9.81f, 0);
            Vector3 velocity = Vector3.zero;

            if (Particles[index].InvMass != 0)
                velocity = Particles[index].Velocity * (1 - Dampening) + (gravity / Particles[index].InvMass);

            PredictedPositions[index] = Particles[index].Position + velocity * TimeStepLength;
        }
    }

    [BurstCompile]
    private struct GetErrorJob : IJobParallelFor
    {
        [ReadOnly] public NativeArray<Particle> Particles;
        [ReadOnly] public NativeArray<DistanceConstraint> Distances;
        [ReadOnly] public NativeArray<Vector3> PredictedPositions;
        public NativeArray<float> Errors;

        public void Execute(int index)
        {
            DistanceConstraint constraint = Distances[index];
            Vector3 grad = PredictedPositions[constraint.ParticleA] - PredictedPositions[constraint.ParticleB];
            Errors[index] = grad.magnitude - constraint.RestLenght;
        }
    }

    [BurstCompile]
    private struct AddJacobi : IJobParallelFor
    {
        [ReadOnly] public NativeArray<DistanceConstraint> Distances;
        [ReadOnly] public NativeArray<Vector3> ResOne;
        [ReadOnly] public NativeArray<Vector3> ResTwo;
        [ReadOnly] public NativeArray<int> PartToConst;

        public NativeArray<Vector3> PredictedPositions;

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

    [BurstCompile]
    private struct SolveConstraintsJob : IJobParallelFor
    {
        [ReadOnly] public NativeArray<Particle> Particles;
        [ReadOnly] public NativeArray<DistanceConstraint> Distances;
        [ReadOnly] public NativeArray<Vector3> PredictedPositions;

        public NativeArray<Vector3> DeltaResOne;
        public NativeArray<Vector3> DeltaResTwo;

        public NativeArray<float> Lambdas;
        public float SubStepLength;

        public void Execute(int index)
        {
            var constraint = Distances[index];
            Vector3 gradient = PredictedPositions[constraint.ParticleA] - PredictedPositions[constraint.ParticleB];
            float distError = gradient.magnitude - constraint.RestLenght;
            gradient.Normalize();

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
    private struct SolveConstraintGroupJob : IJobParallelFor
    {
        [ReadOnly] public NativeArray<Particle> Particles;
        [ReadOnly] public NativeArray<DistanceConstraint> Distances;
        [ReadOnly] public NativeArray<int> PartToConst;

        [ReadOnly] public NativeArray<Vector3> PredictedPositions;
        public NativeArray<Vector3> ResultingPositions;

        public NativeArray<float> Lambdas;
        public float SubStepLength;
        public int GroupToSolve;


        public void Execute(int index)
        {
            if (Particles[index].InvMass == 0)
            {
                ResultingPositions[index] = PredictedPositions[index]; //+ moveDirSum * 0.1f;
                return;
            }

            Vector3 moveDirSum = Vector3.zero;

            for (int i = 0; i < 12; i++)
            {
                Vector3 posSelf = Vector3.zero;
                Vector3 posOther = Vector3.zero;
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

                Vector3 gradient = posOther - posSelf;
                float distError = gradient.magnitude - constraint.RestLenght;

                gradient.Normalize();

                float alpha = constraint.Compliance / (SubStepLength * SubStepLength);

                float invMassOne = selfParticle.InvMass;
                float invMassTwo = otherParticle.InvMass;
                float massSum = invMassOne + invMassTwo;


                float denom = massSum + alpha;
                float dlambda = (-distError - alpha * Lambdas[index]) / denom;

                moveDirSum -= gradient * (invMassOne * dlambda);

                Lambdas[index] += dlambda;
            }

            ResultingPositions[index] = PredictedPositions[index] + moveDirSum * 0.5f;
        }
    }
}