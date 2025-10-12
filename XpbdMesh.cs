using System;
using System.Collections.Generic;
using Parallel_XPBD;
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

    public DistCon DistancesNext;
    public DistCon DistancesShear;
    public DistCon DistancesBend;

    public int[] ParticleToConst;
    private Mesh _mesh;
    private MeshFilter _meshFilter;

    void Awake()
    {
        Reset();
    }

    private void Update()
    {
        if (reset)
        {
            Reset();
        }

        xpbd.Simulate(subSteps);
        // Particles[0].Position = transform.position;

        UpdateMesh();
    }

    private void Reset()
    {
        CreateParticles();
        xpbd = new Xpbd(this);
        xpbd.Dampening = dampening;
        xpbd.SolveParallel = solveParallel;
        SetupMesh();
        reset = false;
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
        _mesh.vertices = new Vector3[xSize * ySize];

        List<int> triangles = new List<int>();
        for (int i = 0; i < Particles.Length; i++)
        {
            if (i < xSize || i % xSize == xSize - 1) continue;

            triangles.Add(i - xSize);
            triangles.Add(i);
            triangles.Add(i + 1);

            triangles.Add(i + 1);
            triangles.Add((i - xSize) + 1);
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

    private DistCon GetNeighborDistanceConstraints()
    {
        DistCon neighbors = new DistCon();

        List<Connection> distanceList = new List<Connection>();
        neighbors.Groups = new int[Particles.Length];

        for (int i = 0; i < Particles.Length; i++)
        {
            int currentColumn = i % xSize;
            int currentRow = (i / xSize);
            if (currentRow % 2 == 0) neighbors.Groups[i] = currentColumn % 2;
            else neighbors.Groups[i] = (currentColumn + 1) % 2;

            if (i >= xSize)
            {
                Connection distTop = new Connection()
                {
                    ParticleOne = i,
                    ParticleTwo = i - xSize,
                    RestLenght = Vector3.Distance(Particles[i].Position, Particles[i - xSize].Position),
                    Compliance = compliance * 0.0000000001f,
                };

                distanceList.Add(distTop);
            }

            if (i % xSize != 0)
            {
                Connection distLeft = new Connection()
                {
                    ParticleOne = i,
                    ParticleTwo = i - 1,
                    RestLenght = Vector3.Distance(Particles[i].Position, Particles[i - 1].Position),
                    Compliance = compliance * 0.0000000001f,
                };
                distanceList.Add(distLeft);
            }
        }

        neighbors.Connections = distanceList.ToArray();
        return neighbors;
    }

    private DistCon GetBendDistanceConstraints()
    {
        DistCon neighbors = new DistCon();

        List<Connection> distanceList = new List<Connection>();
        neighbors.Groups = new int[Particles.Length];

        for (int i = 0; i < Particles.Length; i++)
        {
            int currentColumn = i % xSize;
            int currentRow = ((i + 1) / xSize);
            if (currentRow % 4 < 2)
            {
                neighbors.Groups[i] = 0;
                if (currentColumn % 4 < 2)
                    neighbors.Groups[i] = 1;
            }
            else
            {
                neighbors.Groups[i] = 1;
                if (currentColumn % 4 < 2)
                    neighbors.Groups[i] = 0;
            }


            if (i >= xSize * 2)
            {
                Connection distTop = new Connection()
                {
                    ParticleOne = i,
                    ParticleTwo = i - xSize * 2,
                    RestLenght = Vector3.Distance(Particles[i].Position, Particles[i - xSize * 2].Position),
                    Compliance = bendingCompliance * 0.0000000001f
                };

                distanceList.Add(distTop);
            }

            if (i % xSize < xSize - 2)
            {
                Connection distRight = new Connection()
                {
                    ParticleOne = i,
                    ParticleTwo = i + 2,
                    RestLenght = Vector3.Distance(Particles[i].Position, Particles[i + 2].Position),
                    Compliance = bendingCompliance * 0.0000000001f
                };

                distanceList.Add(distRight);
            }
        }

        neighbors.Connections = distanceList.ToArray();
        return neighbors;
    }

    private DistCon GetShearDistanceConstraints()
    {
        DistCon neighbors = new DistCon();

        List<Connection> distanceList = new List<Connection>();
        neighbors.Groups = new int[Particles.Length];

        for (int i = 0; i < Particles.Length; i++)
        {
            int currentRow = (i / xSize);
            neighbors.Groups[i] = currentRow % 2;

            if (i >= xSize && i % xSize != 0)
            {
                Connection distLeftTop = new Connection()
                {
                    ParticleOne = i,
                    ParticleTwo = i - xSize - 1,
                    RestLenght = Vector3.Distance(Particles[i].Position, Particles[i - xSize - 1].Position),
                    Compliance = sheerCompliance * 0.0000000001f
                };
                distanceList.Add(distLeftTop);
            }

            if (i >= xSize && i % xSize != xSize - 1)
            {
                Connection distRightTop = new Connection()
                {
                    ParticleOne = i,
                    ParticleTwo = i - xSize + 1,
                    RestLenght = Vector3.Distance(Particles[i].Position, Particles[i - xSize + 1].Position),
                    Compliance = sheerCompliance * 0.0000000001f
                };
                distanceList.Add(distRightTop);
            }
        }

        neighbors.Connections = distanceList.ToArray();
        return neighbors;
    }

    private Vector3[] GetParticlePositions()
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
        if (!drawGizmos || Particles == null || Distances == null || xpbd == null) return;

        for (int i = 0; i < Particles.Length; i++)
        {
            // Gizmos.DrawSphere(Particles[i].Position, 0.1f);
            //
            // if (DistancesNext.Groups[i] == 0) Gizmos.color = Color.red;
            // if (DistancesNext.Groups[i] == 1) Gizmos.color = Color.blue;
            // if (DistancesNext.Groups[i] == 2) Gizmos.color = Color.yellow;
            // if (DistancesNext.Groups[i] == 3) Gizmos.color = Color.white;
        }

        for (int i = 0; i < Distances.Length; i++)
        {
            Color color = new Color(Mathf.Abs(xpbd.DistError[i]), 0, 1 - Mathf.Abs(xpbd.DistError[i]), 1);
            
            Gizmos.color = color;
            Gizmos.DrawLine(Particles[Distances[i].ParticleA].Position,
                Particles[Distances[i].ParticleB].Position);

            if (Distances[i].PartAGroup < 4) continue;
            if (Distances[i].PartBGroup < 4) continue;

            // if (Distances[i].PartAGroup == 4)
            // {
            //     Gizmos.color = Color.green;
            // }
            //
            // if (Distances[i].PartAGroup == 5)
            // {
            //     Gizmos.color = Color.magenta;
            // }
            //
            // Gizmos.DrawSphere(Particles[Distances[i].ParticleA].Position, 0.1f);
            //
            // if (Distances[i].PartBGroup == 4)
            // {
            //     Gizmos.color = Color.green;
            // }
            //
            // if (Distances[i].PartBGroup == 5)
            // {
            //     Gizmos.color = Color.magenta;
            // }

            Gizmos.DrawSphere(Particles[Distances[i].ParticleB].Position, 0.1f);

            // if (Distances[i].PartBGroup == 1)
            // {
            //     Gizmos.color = Color.green;
            // }
            // if (Distances[i].PartBGroup == 0)
            // {
            //     Gizmos.color = Color.magenta;
            // }
            //
            // Gizmos.DrawSphere(Particles[Distances[i].ParticleB].Position, 0.1f);
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
    public int ParticleA;
    public int ParticleB;
    public float RestLenght;
    public float Compliance;
    public int PartAGroup;
    public int PartBGroup;
}