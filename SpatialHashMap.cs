using System;
using System.Collections.Generic;
using System.Linq;
using System.Runtime.CompilerServices;
using Unity.Burst;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Jobs;
using Unity.Mathematics;
using UnityEngine;
using UnityEngine.Rendering;

public class SpatialHashMap
{
    public SpatialHashMapEntry[] Entries;
    public float GridSize;

    public void AddEntrys(Sphere[] spheres)
    {
    }

    public int GetMaxCellsPerSphere(float maxRadius, float gridSize)
    {
        int inOneDirection = (int)(((maxRadius * 2f) / gridSize) + 2f);
        return inOneDirection * inOneDirection * inOneDirection;
    }

    public void GetCornerPoint()
    {
    }

    public Vector3[] AccessHashMapParallel(Vector3[] points)
    {
        NativeArray<Vector3> results = new NativeArray<Vector3>(points.Length, Allocator.TempJob);
        NativeArray<Vector3> accessPoints = new NativeArray<Vector3>(points, Allocator.TempJob);
        NativeArray<SpatialHashMapEntry> hashMap = new NativeArray<SpatialHashMapEntry>(Entries, Allocator.TempJob);

        AccessHashMap accessJob = new AccessHashMap()
        {
            ResultingDirection = results,
            AccessPoints = accessPoints,
            HashMap = hashMap,
            GridSize = this.GridSize
        };
        accessJob.Schedule(accessPoints.Length, 32).Complete();

        Vector3[] resultingDirections = new Vector3[results.Length];
        accessJob.ResultingDirection.CopyTo(resultingDirections);
        results.Dispose();
        accessPoints.Dispose();
        hashMap.Dispose();
        return resultingDirections;
    }

    public void SaveGridPositionsParallel(Particle[] particles, float gridSize, Transform transformobj)
    {
        List<Sphere> spheresToInsert = new List<Sphere>();
        for (int i = 0; i < particles.Length; i++)
        {
            spheresToInsert.Add(new Sphere()
            {
                Position = transformobj.TransformPoint(particles[i].Position),
                Radius = 0.8f * transformobj.localScale.x,
            });
        }

        SaveGridPositionsParallel(spheresToInsert, gridSize);
    }

    public void SaveGridPositionsParallel(List<Sphere> spheres, float gridSize)
    {
        GridSize = gridSize;

        var spheresNative = new NativeArray<Sphere>(spheres.ToArray(), Allocator.TempJob);
        int estimatedEntries = spheresNative.Length;
        var myList = new NativeList<SpatialHashMapEntry>(50000, Allocator.TempJob);

        var fillJob = new FillHashMap
        {
            Result = myList.AsParallelWriter(),
            GridSize = gridSize,
            Spheres = spheresNative
        };

        JobHandle fillHandle = fillJob.Schedule(spheresNative.Length, 64);

        var sortJob = new SortJob
        {
            HashMap = myList.AsDeferredJobArray()
        };

        JobHandle sortHandle = sortJob.Schedule(fillHandle);

        sortHandle.Complete();

        var hashArray = myList.AsArray();
        Entries = new SpatialHashMapEntry[hashArray.Length];
        hashArray.CopyTo(Entries);
        myList.Dispose();
        spheresNative.Dispose();
    }


    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private int Hash3U(int x, int y, int z)
    {
        unchecked
        {
            return (x * 73856093 ^ y * 19349663 ^ z * 83492791);
        }
    }

    [BurstCompile]
    private struct AccessHashMap : IJobParallelFor
    {
        public NativeArray<Vector3> ResultingDirection;
        [ReadOnly] public NativeArray<Vector3> AccessPoints;
        [ReadOnly] public NativeArray<SpatialHashMapEntry> HashMap;
        [ReadOnly] public float GridSize;

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        static int Hash3(int x, int y, int z)
        {
            unchecked
            {
                return (x * 73856093 ^ y * 19349663 ^ z * 83492791);
            }
        }

        static int LowerBound(NativeArray<SpatialHashMapEntry> arr, int target)
        {
            int lo = 0;
            int hi = arr.Length;
            while (lo < hi)
            {
                int mid = lo + ((hi - lo) >> 1);
                if (arr[mid].Hash < target) lo = mid + 1;
                else hi = mid;
            }

            return lo;
        }

        // Erste Position mit arr[pos].Hash > target (exklusives Ende)
        static int UpperBound(NativeArray<SpatialHashMapEntry> arr, int target)
        {
            int lo = 0;
            int hi = arr.Length;
            while (lo < hi)
            {
                int mid = lo + ((hi - lo) >> 1);
                if (arr[mid].Hash <= target) lo = mid + 1;
                else hi = mid;
            }

            return lo;
        }

        public void Execute(int index)
        {
            Vector3 result = new Vector3();
            int hash = Hash3(Mathf.FloorToInt(AccessPoints[index].x / GridSize),
                Mathf.FloorToInt(AccessPoints[index].y / GridSize),
                Mathf.FloorToInt(AccessPoints[index].z / GridSize));

            int start = LowerBound(HashMap, hash);
            if (start == HashMap.Length || HashMap[start].Hash != hash)
                return; // kein Eintrag mit diesem Hash

            int end = UpperBound(HashMap, hash); // exklusiv

            for (int i = start; i < end; i++)
            {
                if (hash != HashMap[i].Hash) continue;
                if (Vector3.Distance(AccessPoints[index], HashMap[i].Target.Position) >
                    HashMap[i].Target.Radius) continue;

                Vector3 direction = AccessPoints[index] - (Vector3)HashMap[i].Target.Position;
                float distance = direction.magnitude;
                direction.Normalize();

                result = direction * (HashMap[i].Target.Radius - distance);
            }

            ResultingDirection[index] = result;
        }
    }

    [BurstCompile]
    private struct SortJob : IJob
    {
        public NativeArray<SpatialHashMapEntry> HashMap;

        public void Execute()
        {
            HashMap.Sort();
        }
    }

  //  [BurstCompile]
    private struct FillHashMap : IJobParallelFor
    {
        public NativeList<SpatialHashMapEntry>.ParallelWriter Result;
        public float GridSize;

        [ReadOnly] public NativeArray<Sphere> Spheres;

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        static int Hash3(int x, int y, int z)
        {
            unchecked
            {
                return (x * 73856093 ^ y * 19349663 ^ z * 83492791);
            }
        }

        public void Execute(int index)
        {
            float radius = Spheres[index].Radius;
            Vector3 center = Spheres[index].Position;
            float3 origin = new float3(0, 0, 0);

            Vector3 min = center - new Vector3(radius, radius, radius);
            Vector3 max = center + new Vector3(radius, radius, radius);

            int ix0 = Mathf.FloorToInt((min.x) / GridSize);
            int iy0 = Mathf.FloorToInt((min.y) / GridSize);
            int iz0 = Mathf.FloorToInt((min.z) / GridSize);

            int ix1 = Mathf.CeilToInt((max.x) / GridSize) - 1;
            int iy1 = Mathf.CeilToInt((max.y) / GridSize) - 1;
            int iz1 = Mathf.CeilToInt((max.z) / GridSize) - 1;

            float r2 = radius * radius;

            for (int iz = iz0; iz <= iz1; iz++)
            for (int iy = iy0; iy <= iy1; iy++)
            for (int ix = ix0; ix <= ix1; ix++)
            {
                Vector3 cellMin = origin + new float3(ix, iy, iz) * GridSize;
                Vector3 cellMax = cellMin + Vector3.one * GridSize;

                float dx = 0f;
                if (center.x < cellMin.x) dx = cellMin.x - center.x;
                else if (center.x > cellMax.x) dx = center.x - cellMax.x;

                float dy = 0f;
                if (center.y < cellMin.y) dy = cellMin.y - center.y;
                else if (center.y > cellMax.y) dy = center.y - cellMax.y;

                float dz = 0f;
                if (center.z < cellMin.z) dz = cellMin.z - center.z;
                else if (center.z > cellMax.z) dz = center.z - cellMax.z;

                float dist2 = dx * dx + dy * dy + dz * dz;

                if (dist2 <= r2)
                {
                    SpatialHashMapEntry entry = new SpatialHashMapEntry();
                    entry.Hash = Hash3(ix, iy, iz);
                    Sphere targetSphere = new Sphere();
                    targetSphere.Position = Spheres[index].Position;
                    targetSphere.Radius = Spheres[index].Radius;
                    entry.Target = targetSphere;
                    entry.Corner = cellMin;
                    Result.AddNoResize(entry);
                }
            }
        }
    }
}


public struct SpatialHashMapEntry : IComparable<SpatialHashMapEntry>
{
    public int Hash;
    public Sphere Target;
    public Vector3 Corner;

    public int CompareTo(SpatialHashMapEntry other)
    {
        return Hash.CompareTo(other.Hash);
    }
}