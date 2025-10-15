using Parallel_XPBD;
using Unity.Burst;
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;


public class Xpbd
{
    private XpbdMesh _toSimulate;
    public float TimeStepLength = 1f / 200;
    public float Dampening;

    public float3[] ParticlePositions;
    public float3[] oldPositions;

    private SerialXpbdSolver _serialXpbdSolver;
    private JacobiXpbdSolver _jacobiXpbdSolver;
    private ColorGroupXpbdSolver _colorGroupXpbdSolver;
    private SerialParticleSolver _serialParticleSolver;
    private ISoftBodySolver _solver;

    public enum Solver
    {
        Serial,
        SerialPerParticle,
        Jacobi,
        ParticleColorGroups
    }

    public Xpbd(XpbdMesh toSimulate)
    {
        _toSimulate = toSimulate;
        ParticlePositions = new float3[_toSimulate.Particles.Length];
        _serialXpbdSolver = new SerialXpbdSolver(_toSimulate);
        _jacobiXpbdSolver = new JacobiXpbdSolver(_toSimulate);
        _colorGroupXpbdSolver = new ColorGroupXpbdSolver(_toSimulate);
    }

    public void SetSolver(Solver solver)
    {
        switch (solver)
        {
            case Solver.Serial:
                _solver = _serialXpbdSolver;
                break;
            case Solver.Jacobi:
                _solver = _jacobiXpbdSolver;
                break;
            case Solver.ParticleColorGroups:
                _solver = _colorGroupXpbdSolver;
                break;
            case Solver.SerialPerParticle:
                _solver = _serialParticleSolver;
                break;
        }
    }

    public void Simulate(int subSteps)
    {
        Integrate();
        _solver.SolveDistanceConstraints(TimeStepLength, subSteps, ref ParticlePositions);
        UpdatePartícles();
    }

    private void Integrate()
    {
        NativeArray<Particle> nativeParticles = new NativeArray<Particle>(_toSimulate.Particles, Allocator.TempJob);
        NativeArray<float3> nativePredictedPositions =
            new NativeArray<float3>(ParticlePositions.Length, Allocator.TempJob);
       
        IntegrateJob integrationJob = new IntegrateJob()
        {
            Dampening = Dampening,
            TimeStepLength = TimeStepLength,
            PredictedPositions = nativePredictedPositions,
            Particles = nativeParticles
        };
        integrationJob.Schedule(_toSimulate.Particles.Length, 32).Complete();
        nativePredictedPositions.CopyTo(ParticlePositions);

        nativeParticles.Dispose();
        nativePredictedPositions.Dispose();
    }

    private void UpdatePartícles()
    {
        NativeArray<Particle> nativeParticles = new NativeArray<Particle>(_toSimulate.Particles, Allocator.TempJob);
        NativeArray<float3> nativePredictedPositions = new NativeArray<float3>(ParticlePositions, Allocator.TempJob);

        PositionUpdateJob updateJob = new PositionUpdateJob()
        {
            Particles = nativeParticles,
            PredictedPositions = nativePredictedPositions,
            TimeStepLength = TimeStepLength,
        };
        updateJob.Schedule(_toSimulate.Particles.Length, 32).Complete();

        nativeParticles.CopyTo(_toSimulate.Particles);
        nativeParticles.Dispose();
        nativePredictedPositions.Dispose();
    }


    [BurstCompile]
    private struct PositionUpdateJob : IJobParallelFor
    {
        public NativeArray<Particle> Particles;
        [ReadOnly] public NativeArray<float3> PredictedPositions;
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
        public NativeArray<float3> PredictedPositions;
        public float Dampening;
        public float TimeStepLength;

        public void Execute(int index)
        {
            float3 gravity = new float3(0, -9.81f, 0);
            float3 velocity = float3.zero;

            if (Particles[index].InvMass != 0)
                velocity = Particles[index].Velocity * (1 - Dampening) + (gravity / Particles[index].InvMass);

            PredictedPositions[index] = Particles[index].Position + velocity * TimeStepLength;
        }
    }
}