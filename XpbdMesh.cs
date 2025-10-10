using System;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Serialization;
using Random = UnityEngine.Random;

public class XpbdMesh : MonoBehaviour
{
    public Xpbd xpbd;
    public int xSize = 20;
    public int ySize = 20;
    public bool reset;
    public bool solveParallel;
    public bool drawGizmos;

    [Range(1, 200)] public int subSteps = 50;
    [Range(0f, 1f)] public float dampening = 0.05f;
    [Range(0f, 10000f)] public float compliance = 0.000000001f;
    [Range(0f, 10000f)] public float sheerCompliance = 0.000000001f;
    [Range(0f, 10000f)] public float bendingCompliance = 0.000000001f;

    [Range(0f, 1 / 50f)] public float timeStep = 1 / 20000f;

    public Particle[] Particles;
    public DistanceConstraint[] Distances;
    public int[] ParticleToConst;
    private Mesh _mesh;
    private MeshFilter _meshFilter;

    void Awake()
    {
        CreateParticles();
        xpbd = new Xpbd(this);
        xpbd.Dampening = dampening;
        xpbd.SolveParallel = solveParallel;
        SetupMesh();
    }

    private void Update()
    {
        if (reset)
        {
            CreateParticles();
            reset = false;
        }

        for (int i = 0; i < Particles.Length; i++)
        {
            
        }
        
        xpbd.Simulate(subSteps);
        Particles[0].Position = transform.position;

        UpdateMesh();
    }

    private void OnValidate()
    {
        if (xpbd == null) return;
        xpbd.Dampening = dampening;
        xpbd.TimeStepLength = timeStep;
        xpbd.SolveParallel = solveParallel;
    }

    private void SetupMesh()
    {
        _meshFilter = GetComponent<MeshFilter>();
        if (_meshFilter == null) return;
        _mesh = new Mesh();
        _meshFilter.mesh = _mesh;
        _mesh.vertices = new Vector3[xSize*ySize];
        
        List<int> triangles = new List<int>();
        for (int i = 0; i < Particles.Length; i++)
        {
            if (i < xSize || i % xSize == xSize-1) continue;
            
            triangles.Add(i - xSize);
            triangles.Add(i);
            triangles.Add(i + 1);

            triangles.Add(i + 1);
            triangles.Add((i - xSize)+1);
            triangles.Add(i - xSize);
        }
        _mesh.triangles = triangles.ToArray();
    }

    private void UpdateMesh()
    {
        if (_meshFilter == null) return;
        if (_mesh == null)
        {
            _mesh = new Mesh();
            _meshFilter.mesh = _mesh;
        }
        
        _mesh.vertices = xpbd.CurrentParticlePositions;
        _mesh.RecalculateNormals();
    }

    private void CreateParticles()
    {
        Particles = new Particle[xSize * ySize];
        ParticleToConst = new int[12 * ySize * xSize];
        for (int i = 0; i < ParticleToConst.Length; i++)
        {
            ParticleToConst[i] = -1;
        }

        Vector3[] positions = GetParticelPositions();
        Vector3[] velocitys = new Vector3[Particles.Length];

        for (int i = 0; i < Particles.Length; i++)
        {
            float invMass = 1f;
            if (i == 0 || i == xSize - 1) invMass = 0f;
            Particles[i] = new Particle()
            {
                Position = positions[i],
                Velocity = velocitys[i],
                InvMass = invMass
            };
        }

        Distances = GetDistanceConstraints();
    }

    private DistanceConstraint[] GetDistanceConstraints()
    {
        List<DistanceConstraint> distanceList = new List<DistanceConstraint>();

        for (int i = 0; i < Particles.Length; i++)
        {
            if (i >= xSize)
            {
                DistanceConstraint distTop = new DistanceConstraint()
                {
                    ParticleOne = i,
                    ParticleTwo = i - xSize,
                    RestLenght = Vector3.Distance(Particles[i].Position, Particles[i - xSize].Position),
                    Compliance = compliance * 0.0000000001f
                };
                ParticleToConst[i * 12] = distanceList.Count;
                ParticleToConst[(i - xSize) * 12 + 1] = distanceList.Count;
                distanceList.Add(distTop);
            }

            if (i % xSize != 0)
            {
                DistanceConstraint distLeft = new DistanceConstraint()
                {
                    ParticleOne = i,
                    ParticleTwo = i - 1,
                    RestLenght = Vector3.Distance(Particles[i].Position, Particles[i - 1].Position),
                    Compliance = compliance * 0.0000000001f
                };
                ParticleToConst[i * 12 + 2] = distanceList.Count;
                ParticleToConst[(i - 1) * 12 + 3] = distanceList.Count;
                distanceList.Add(distLeft);
            }

            if (i >= xSize && i % xSize != 0)
            {
                DistanceConstraint distLeftTop = new DistanceConstraint()
                {
                    ParticleOne = i,
                    ParticleTwo = i - xSize - 1,
                    RestLenght = Vector3.Distance(Particles[i].Position, Particles[i - xSize - 1].Position),
                    Compliance = sheerCompliance * 0.0000000001f
                };
                ParticleToConst[i * 12 + 4] = distanceList.Count;
                ParticleToConst[(i - xSize - 1) * 12 + 5] = distanceList.Count;
                distanceList.Add(distLeftTop);
            }

            if (i >= xSize && i % xSize != xSize - 1)
            {
                DistanceConstraint distRightTop = new DistanceConstraint()
                {
                    ParticleOne = i,
                    ParticleTwo = i - xSize + 1,
                    RestLenght = Vector3.Distance(Particles[i].Position, Particles[i - xSize + 1].Position),
                    Compliance = sheerCompliance * 0.0000000001f
                };

                ParticleToConst[i * 12 + 6] = distanceList.Count;
                ParticleToConst[(i - xSize + 1) * 12 + 7] = distanceList.Count;
                distanceList.Add(distRightTop);
            }

            if (i >= xSize * 2)
            {
                DistanceConstraint distLeftRight = new DistanceConstraint()
                {
                    ParticleOne = i,
                    ParticleTwo = i - xSize * 2,
                    RestLenght = Vector3.Distance(Particles[i].Position, Particles[i - xSize * 2].Position),
                    Compliance = bendingCompliance * 0.0000000001f
                };
                ParticleToConst[i * 12 + 8] = distanceList.Count;
                ParticleToConst[(i - xSize * 2) * 12 + 9] = distanceList.Count;
                distanceList.Add(distLeftRight);
            }

            if (i % xSize < xSize - 2)
            {
                DistanceConstraint distLeftRight = new DistanceConstraint()
                {
                    ParticleOne = i,
                    ParticleTwo = i + 2,
                    RestLenght = Vector3.Distance(Particles[i].Position, Particles[i + 2].Position),
                    Compliance = bendingCompliance * 0.0000000001f
                };
                ParticleToConst[i * 12 + 10] = distanceList.Count;
                ParticleToConst[(i + 2) * 12 + 11] = distanceList.Count;
                distanceList.Add(distLeftRight);
            }
        }

        return distanceList.ToArray();
    }

    private Vector3[] GetParticelPositions()
    {
        Vector3 origin = transform.position;

        Vector3[] positions = new Vector3[xSize * ySize];

        for (int x = 0; x < xSize; x++)
        {
            for (int y = 0; y < ySize; y++)
            {
                positions[y * xSize + x] = origin + new Vector3(x, -y, Random.Range(0f, 0.001f));
            }
        }

        return positions;
    }

    private void OnDrawGizmos()
    {
        if (!drawGizmos || Particles == null || Distances == null || xpbd == null ) return;

        for (int i = 0; i < Particles.Length; i++)
        {
            Gizmos.DrawSphere(Particles[i].Position, 0.1f);
        }

        for (int i = 0; i < Distances.Length; i++)
        {
            Color color = new Color(Mathf.Abs(xpbd.DistError[i]), 0, 1 - Mathf.Abs(xpbd.DistError[i]), 1);
            Gizmos.color = color;
            Gizmos.DrawLine(Particles[Distances[i].ParticleOne].Position, Particles[Distances[i].ParticleTwo].Position);
        }
    }
}

public struct Particle
{
    public float InvMass;
    public Vector3 Position;
    public Vector3 Velocity;
}

public struct DistanceConstraint
{
    public int ParticleOne;
    public int ParticleTwo;
    public float RestLenght;
    public float Compliance;
}