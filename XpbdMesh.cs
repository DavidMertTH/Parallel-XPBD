using System;
using System.Collections.Generic;
using Parallel_XPBD;
using Parallel_XPBD.Collisions;
using Unity.Jobs.LowLevel.Unsafe;
using Unity.Mathematics;
using UnityEngine;
using UnityEngine.Serialization;
using Random = UnityEngine.Random;

public class XpbdMesh : MonoBehaviour
{
    public Xpbd xpbd;
    public int xSize = 20;
    public int ySize = 20;
    public bool reset;
    public bool drawGizmos;
    //public Xpbd.Solver solver;
    public bool handleCollisions;
    public bool sweepVolume;
    
    [Range(0, 1)] public float collisionResolution = 1;
    [Range(1, 400)] public int subSteps = 50;
    [Range(0f, 1f)] public float dampening = 0.05f;
    [Range(0f, 10000f)] public float compliance = 0.000000001f;
    [Range(0f, 10000f)] public float sheerCompliance = 0.000000001f;
    [Range(0f, 10000f)] public float bendingCompliance = 0.000000001f;
    [Range(0f, 1 / 50f)] public float timeStep = 1 / 20000f;

    public Particle[] Particles;
    public DistanceConstraint[] Distances;
    public event Action Destroyed;
    [HideInInspector] public int[] ParticleToConst;

    private Mesh _mesh;
    private MeshFilter _meshFilter;


    void Start()
    {
        Reset(false);
        JobsUtility.JobDebuggerEnabled = false;
    }

    private void LateUpdate()
    {
        if (reset)
        {
            Reset(true);
        }

        xpbd.Simulate(subSteps);
        UpdateMesh();
    }

    private void OnValidate()
    {
        if (xpbd == null) return;
    }

    private void Reset(bool stopOld)
    {
        CreateParticles();
        if (stopOld) xpbd.DisposeEverything();
        xpbd = new Xpbd(this);
        xpbd.handleCollisions = handleCollisions;
        xpbd.Dampening = dampening;
        xpbd.TimeStepLength = this.timeStep;
        SetupMesh();
        reset = false;
        //transform.localScale = Vector3.one / xSize * 30;
    }

    private void OnDestroy()
    {
        Destroyed?.Invoke();
    }

    private void SetupMesh()
    {
        _meshFilter = GetComponent<MeshFilter>();
        if (_meshFilter == null) return;
        _mesh = new Mesh();
        _meshFilter.mesh = _mesh;
        _mesh.vertices = new Vector3[xSize * ySize];
        Vector2[] uvs = new Vector2[xSize * ySize];

        List<int> triangles = new List<int>();
        for (int i = 0; i < Particles.Length; i++)
        {
            int xPos = i % xSize;
            int yPos = i / xSize;

            uvs[i] = new Vector2((float)xPos / xSize, (float)yPos / ySize);

            if (i < xSize || i % xSize == xSize - 1) continue;
            if (!Particles[i].IsActive || !Particles[i - xSize].IsActive || !Particles[i + 1].IsActive ||
                !Particles[i - xSize + 1].IsActive) continue;
            triangles.Add(i - xSize);
            triangles.Add(i);
            triangles.Add(i + 1);

            triangles.Add(i + 1);
            triangles.Add((i - xSize) + 1);
            triangles.Add(i - xSize);
        }

        _mesh.triangles = triangles.ToArray();
        _mesh.uv = uvs;
        _mesh.uv2 = uvs;
    }

    private void UpdateMesh()
    {
        if (_meshFilter == null) return;
        if (_mesh == null)
        {
            _mesh = new Mesh();
            _meshFilter.mesh = _mesh;
        }


        _mesh.vertices = Helper.FloatToVectorParallel(xpbd.ParticlePositions);
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

        Vector3[] positions = GetParticlePositions();
        Vector3[] velocitys = new Vector3[Particles.Length];

        for (int i = 0; i < Particles.Length; i++)
        {
            float invMass = 1f;
            if (i == 0 || i == xSize - 1) invMass = 0f;
            Particles[i] = new Particle()
            {
                Position = positions[i],
                Velocity = velocitys[i],
                InvMass = invMass,
                IsActive = true
            };
        }

        Distances = GetDistanceConstraints();
    }

    private void CreateParticlesForPoncho()
    {
        Particles = new Particle[xSize * ySize];
        ParticleToConst = new int[12 * ySize * xSize];
        for (int i = 0; i < ParticleToConst.Length; i++)
        {
            ParticleToConst[i] = -1;
        }

        Vector3[] positions = GetParticlePositions();
        Vector3[] velocitys = new Vector3[Particles.Length];

        for (int i = 0; i < Particles.Length; i++)
        {
            float invMass = 1f;
            //if (i == 0 || i == xSize - 1) invMass = 0f;

            int xPos = i % xSize;
            int yPos = i / xSize;
            bool isInXRange = xPos > (xSize / 2) - (int)((float)xSize / 15) &&
                              xPos < (xSize / 2) + (int)((float)xSize / 15);
            bool isInYRange = yPos > (ySize / 2) - (int)((float)ySize / 15) &&
                              yPos < (ySize / 2) + (int)((float)ySize / 15);
            bool activeParticle = !(isInXRange && isInYRange);

            Particles[i] = new Particle()
            {
                Position = positions[i],
                Velocity = velocitys[i],
                InvMass = invMass,
                IsActive = activeParticle
            };
        }

        Distances = GetDistanceConstraints();
    }

    private DistanceConstraint[] GetDistanceConstraints()
    {
        List<DistanceConstraint> distanceList = new List<DistanceConstraint>();

        for (int i = 0; i < Particles.Length; i++)
        {
            int groupA = -1;
            int groupB = -1;

            if (i >= xSize)
            {
                if (Row(i) % 2 == 0)
                {
                    if (Column(i) % 2 == 0) groupA = 0;
                    else groupA = 1;
                }
                else
                {
                    if (Column(i) % 2 == 0) groupA = 1;
                    else groupA = 0;
                }

                groupB = 1 - groupA;

                DistanceConstraint distTop = new DistanceConstraint()
                {
                    ParticleA = i,
                    ParticleB = i - xSize,
                    RestLenght = Vector3.Distance(Particles[i].Position, Particles[i - xSize].Position),
                    Compliance = compliance * 0.0000000001f,
                    PartAGroup = groupA,
                    PartBGroup = groupB
                };
                ParticleToConst[i * 12] = distanceList.Count;
                ParticleToConst[(i - xSize) * 12 + 1] = distanceList.Count;
                distanceList.Add(distTop);
            }

            if (i % xSize != 0)
            {
                DistanceConstraint distLeft = new DistanceConstraint()
                {
                    ParticleA = i,
                    ParticleB = i - 1,
                    RestLenght = Vector3.Distance(Particles[i].Position, Particles[i - 1].Position),
                    Compliance = compliance * 0.0000000001f,
                    PartAGroup = groupA,
                    PartBGroup = groupB
                };
                ParticleToConst[i * 12 + 2] = distanceList.Count;
                ParticleToConst[(i - 1) * 12 + 3] = distanceList.Count;
                distanceList.Add(distLeft);
            }

            if (Row(i) % 2 == 0)
            {
                groupA = 2;
                groupB = 3;
            }
            else
            {
                groupA = 3;
                groupB = 2;
            }

            if (i >= xSize && i % xSize != 0)
            {
                DistanceConstraint distLeftTop = new DistanceConstraint()
                {
                    ParticleA = i,
                    ParticleB = i - xSize - 1,
                    RestLenght = Vector3.Distance(Particles[i].Position, Particles[i - xSize - 1].Position),
                    Compliance = sheerCompliance * 0.0000000001f,
                    PartAGroup = groupA,
                    PartBGroup = groupB
                };
                ParticleToConst[i * 12 + 4] = distanceList.Count;
                ParticleToConst[(i - xSize - 1) * 12 + 5] = distanceList.Count;
                distanceList.Add(distLeftTop);
            }

            if (i >= xSize && i % xSize != xSize - 1)
            {
                DistanceConstraint distRightTop = new DistanceConstraint()
                {
                    ParticleA = i,
                    ParticleB = i - xSize + 1,
                    RestLenght = Vector3.Distance(Particles[i].Position, Particles[i - xSize + 1].Position),
                    Compliance = sheerCompliance * 0.0000000001f,
                    PartAGroup = groupA,
                    PartBGroup = groupB
                };

                ParticleToConst[i * 12 + 6] = distanceList.Count;
                ParticleToConst[(i - xSize + 1) * 12 + 7] = distanceList.Count;
                distanceList.Add(distRightTop);
            }

            if (Row(i) % 4 < 2)
            {
                if (Column(i) % 4 < 2)
                {
                    groupA = 4;
                    groupB = 5;
                }
                else
                {
                    groupA = 5;
                    groupB = 4;
                }
            }
            else
            {
                if (Column(i) % 4 < 2)
                {
                    groupA = 5;
                    groupB = 4;
                }
                else
                {
                    groupA = 4;
                    groupB = 5;
                }
            }

            if (i >= xSize * 2)
            {
                DistanceConstraint distLeftRight = new DistanceConstraint()
                {
                    ParticleA = i,
                    ParticleB = i - xSize * 2,
                    RestLenght = Vector3.Distance(Particles[i].Position, Particles[i - xSize * 2].Position),
                    Compliance = bendingCompliance * 0.0000000001f,
                    PartAGroup = groupA,
                    PartBGroup = groupB
                };
                ParticleToConst[i * 12 + 8] = distanceList.Count;
                ParticleToConst[(i - xSize * 2) * 12 + 9] = distanceList.Count;
                distanceList.Add(distLeftRight);
            }

            if (i % xSize < xSize - 2)
            {
                DistanceConstraint distLeftRight = new DistanceConstraint()
                {
                    ParticleA = i,
                    ParticleB = i + 2,
                    RestLenght = Vector3.Distance(Particles[i].Position, Particles[i + 2].Position),
                    Compliance = bendingCompliance * 0.0000000001f,
                    PartAGroup = groupA,
                    PartBGroup = groupB
                };
                ParticleToConst[i * 12 + 10] = distanceList.Count;
                ParticleToConst[(i + 2) * 12 + 11] = distanceList.Count;
                distanceList.Add(distLeftRight);
            }
        }

        return distanceList.ToArray();
    }

    private int Row(int index)
    {
        return (index) / xSize;
    }


    private int Column(int index)
    {
        return index % xSize;
    }


    private Vector3[] GetParticlePositions()
    {
        Vector3 origin = transform.position;

        Vector3[] positions = new Vector3[xSize * ySize];

        for (int x = 0; x < xSize; x++)
        {
            for (int y = 0; y < ySize; y++)
            {
                positions[y * xSize + x] = origin + new Vector3(x, Random.Range(0f, 0.001f), y);
            }
        }

        return positions;
    }


    private void OnDrawGizmos()
    {
        if (!drawGizmos || Particles == null || Distances == null || xpbd == null) return;

        for (int i = 0; i < Distances.Length; i++)
        {
            Gizmos.DrawLine(Particles[Distances[i].ParticleA].Position, Particles[Distances[i].ParticleB].Position);
        }

        for (int i = 0; i < Particles.Length; i++)
        {
            if (Particles[i].IsActive) Gizmos.color = Color.green;
            else Gizmos.color = Color.red;
            Gizmos.DrawSphere(Particles[i].Position, 0.1f);
        }
    }
}

public struct Particle
{
    public float InvMass;
    public float3 Position;
    public float3 Velocity;
    public bool IsActive;
}

public struct DistanceConstraint
{
    public int ParticleA;
    public int ParticleB;
    public float RestLenght;
    public float Compliance;
    public int PartAGroup;
    public int PartBGroup;
}