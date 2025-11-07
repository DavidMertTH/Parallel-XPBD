using Unity.Mathematics;

namespace Parallel_XPBD
{
    public interface ISoftBodySolver
    {
        public void SolveConstraints(float timeStepLength, int subSteps, ref float3[] predictedPositions);
    }
}