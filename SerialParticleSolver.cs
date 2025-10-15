using Unity.Mathematics;

namespace Parallel_XPBD
{
    public class SerialParticleSolver : ISoftBodySolver
    {
        public void SolveDistanceConstraints(float timeStepLength, int subSteps, ref float3[] predictedPositions)
        {
            throw new System.NotImplementedException();
        }
    }
}