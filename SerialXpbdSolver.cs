using Unity.Mathematics;
using UnityEngine;

namespace Parallel_XPBD
{
    public class SerialXpbdSolver: ISoftBodySolver
    {
        public float3[] Gradients;
        public float[] Lambdas;
        private XpbdMesh _toSimulate;

        public SerialXpbdSolver(XpbdMesh xpbdMesh)
        {
            _toSimulate = xpbdMesh;
            Gradients = new float3[_toSimulate.Distances.Length];
            Lambdas = new float[_toSimulate.Distances.Length];
        }

        public void SolveDistanceConstraints(float timeStepLength, int subSteps, ref float3[] predictedPositions)
        {
            float subStepLength = timeStepLength / subSteps;


            for (int s = 0; s < subSteps; s++)
            {
                DistanceConstraint constraint;

                for (int i = 0; i < _toSimulate.Distances.Length; i++)
                {
                    constraint = _toSimulate.Distances[i];
                    Gradients[i] = predictedPositions[constraint.ParticleA] -
                                   predictedPositions[constraint.ParticleB];
                    float error  = math.length(Gradients[i]) - constraint.RestLenght;
                    Gradients[i] = math.normalize(Gradients[i]);

                    float alpha = constraint.Compliance / (subStepLength * subStepLength);

                    float invMassOne = _toSimulate.Particles[constraint.ParticleA].InvMass;
                    float invMassTwo = _toSimulate.Particles[constraint.ParticleB].InvMass;
                    float massSum = invMassOne + invMassTwo;


                    float denom = massSum + alpha;
                    float dlambda = (-error - alpha * Lambdas[i]) / denom;
                    if (invMassOne > 0f)
                        predictedPositions[constraint.ParticleA] += Gradients[i] * (invMassOne * dlambda);

                    if (invMassTwo > 0f)
                        predictedPositions[constraint.ParticleB] += -Gradients[i] * (invMassTwo * dlambda);

                    Lambdas[i] += dlambda;
                }
            }
        }
    }
}