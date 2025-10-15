using Unity.Mathematics;

namespace Parallel_XPBD
{
    public interface ISoftBodySolver
    {
        public void SolveDistanceConstraints(float timeStepLength, int subSteps, ref float3[] predictedPositions);
    }
}