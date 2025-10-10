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
    public bool SolveParallel;

    public Vector3[] CurrentParticlePositions;

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
        if (SolveParallel) SolveConstraintsParallel(subSteps);
        else SolveConstraints(subSteps);
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
                Gradients[i] = PredictedPositions[constraint.ParticleOne] - PredictedPositions[constraint.ParticleTwo];
                DistError[i] = Gradients[i].magnitude - constraint.RestLenght;
                Gradients[i].Normalize();
                // Gradients[i] *= DistError[i];

                float alpha = constraint.Compliance / (subStepLength * subStepLength);

                float invMassOne = _toSimulate.Particles[constraint.ParticleOne].InvMass;
                float invMassTwo = _toSimulate.Particles[constraint.ParticleTwo].InvMass;
                float massSum = invMassOne + invMassTwo;


                float denom = massSum + alpha;
                float dlambda = (-DistError[i] - alpha * Lambdas[i]) / denom;

                if (invMassOne > 0f)
                    PredictedPositions[constraint.ParticleOne] += Gradients[i] * (invMassOne * dlambda);

                if (invMassTwo > 0f)
                    PredictedPositions[constraint.ParticleTwo] += -Gradients[i] * (invMassTwo * dlambda);

                Lambdas[i] += dlambda;
            }
        }
    }


    private void UpdatePartícles()
    {
        NativeArray<Particle> nativeParticles = new NativeArray<Particle>(_toSimulate.Particles, Allocator.TempJob);
        NativeArray<Vector3> nativePredictedPositions = new NativeArray<Vector3>(PredictedPositions, Allocator.TempJob);

        PositionUpdateJob updateJob = new PositionUpdateJob()
        {
            Particles = nativeParticles,
            PredictedPositions = nativePredictedPositions,
            TimeStepLength = TimeStepLength,
        };
        updateJob.Schedule(_toSimulate.Particles.Length, 32).Complete();

        if (CurrentParticlePositions == null) CurrentParticlePositions = new Vector3[_toSimulate.Particles.Length];

        nativePredictedPositions.CopyTo(CurrentParticlePositions);
        for (int i = 0; i < CurrentParticlePositions.Length; i++)
        {
            CurrentParticlePositions[i] = _toSimulate.transform.TransformPoint(CurrentParticlePositions[i]);
        }       
        
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
        NativeArray<Vector3> nativePredictedPositions = new NativeArray<Vector3>(PredictedPositions, Allocator.TempJob);

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
            Vector3 grad = PredictedPositions[constraint.ParticleOne] - PredictedPositions[constraint.ParticleTwo];
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
                if (constrain.ParticleOne == index)
                {
                    PredictedPositions[index] += ResOne[constrainIndex] * 0.3f;
                }

                if (constrain.ParticleTwo == index)
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
            Vector3 gradient = PredictedPositions[constraint.ParticleOne] - PredictedPositions[constraint.ParticleTwo];
            float distError = gradient.magnitude - constraint.RestLenght;
            gradient.Normalize();

            float alpha = constraint.Compliance / (SubStepLength * SubStepLength);

            float invMassOne = Particles[constraint.ParticleOne].InvMass;
            float invMassTwo = Particles[constraint.ParticleTwo].InvMass;
            float massSum = invMassOne + invMassTwo;


            float denom = massSum + alpha;
            float dlambda = (-distError - alpha * Lambdas[index]) / denom;

            DeltaResOne[index] = gradient * (invMassOne * dlambda);
            DeltaResTwo[index] = -gradient * (invMassTwo * dlambda);

            Lambdas[index] += dlambda;
        }
    }
}