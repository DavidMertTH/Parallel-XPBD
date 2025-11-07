using System;
using System.Collections.Generic;
using System.Linq;
using System.Runtime.CompilerServices;
using myXpbd.Parallel_XPBD.Collisions;
using Unity.Burst;
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;
using UnityEngine;

namespace Parallel_XPBD.Collisions
{
    public class SphereMap
    {
        public SpatialHashMapEntry[] Entries;
        public float GridSize;
        private bool _hashMapIsInitialized;
        private NativeArray<SpatialHashMapEntry> _hashMap;
        private JobHandle _fillJobHandle;
        private NativeList<SpatialHashMapEntry> _entryNativeList;
        private FillHashMap _fillJob;
        private bool _fillChainIsRunning;

        public SphereMap()
        {
            Entries = Array.Empty<SpatialHashMapEntry>();
            _fillChainIsRunning = false;
        }

        public void DisposeHashMap()
        {
            if (_hashMapIsInitialized)
            {
                _hashMapIsInitialized = false;
                _hashMap.Dispose();
            }
        }

        public void OnDestroy()
        {
            CompleteHashMapFill();
        }

        [BurstCompile]
        private struct ToLocalJob : IJobFor
        {
            [ReadOnly] public NativeArray<Sphere> InSpheres;
            public NativeArray<Sphere> OutSpheres;

            public float4x4 WorldToLocal;
            public float InvScaleX;

            public void Execute(int i)
            {
                var s = InSpheres[i];

                float4 p4 = math.mul(WorldToLocal, new float4(s.Position, 1f));
                float3 p = p4.xyz;

                float r = s.Radius * InvScaleX;

                OutSpheres[i] = new Sphere { Position = p, Radius = r };
            }
        }

        public void EnterSpheres(NativeArray<Sphere> spheresWorld, XpbdMesh toSimulate, float gridSize)
        {
            int count = spheresWorld.Length;
            if (count == 0) return;

            var tr = toSimulate.transform;
            float4x4 worldToLocal = tr.worldToLocalMatrix;
            float invScaleX = 1f / tr.localScale.x;
            NativeArray<Sphere> spheresOut = new NativeArray<Sphere>(spheresWorld, Allocator.TempJob);
            var job = new ToLocalJob
            {
                InSpheres = spheresWorld,
                OutSpheres = spheresOut,
                WorldToLocal = worldToLocal,
                InvScaleX = invScaleX
            };

            job.ScheduleParallel(count, 64, default).Complete();

            float gridSizeLocal = gridSize * invScaleX;

            SaveGridPositionsParallel(spheresOut, gridSizeLocal);
        }

        public JobHandle AccessHashMapParallel(NativeArray<float3> positions, JobHandle previousJob)
        {
            if (!_hashMapIsInitialized)
            {
                _hashMap = new NativeArray<SpatialHashMapEntry>(Entries, Allocator.TempJob);
                _hashMapIsInitialized = true;
            }

            AccessHashMap accessJob = new AccessHashMap()
            {
                AccessPoints = positions,
                HashMap = _hashMap,
                GridSize = this.GridSize
            };
            return accessJob.Schedule(positions.Length, 32, previousJob);
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

            SaveGridPositionsParallel(spheresToInsert.ToArray(), gridSize);
        }

        public void SaveGridPositionsParallel(Sphere[] spheres, float gridSize)
        {
            NativeArray<Sphere> nativeSpheres = new NativeArray<Sphere>(spheres, Allocator.TempJob);
            SaveGridPositionsParallel(nativeSpheres, gridSize);
        }

        public void SaveGridPositionsParallel(NativeArray<Sphere> spheres, float gridSize)
        {
            if (_fillChainIsRunning) CompleteHashMapFill();
            _entryNativeList = new NativeList<SpatialHashMapEntry>(50000, Allocator.TempJob);
            _fillChainIsRunning = true;
            GridSize = gridSize;
            _fillJob = new FillHashMap
            {
                Result = _entryNativeList.AsParallelWriter(),
                GridSize = gridSize,
                Spheres = spheres
            };

            JobHandle fillHandle = _fillJob.Schedule(spheres.Length, 64);

            var sortJob = new SortJob
            {
                HashMap = _entryNativeList.AsDeferredJobArray()
            };

            _fillJobHandle = sortJob.Schedule(fillHandle);
        }

        private void CompleteHashMapFill()
        {
            if (!_fillChainIsRunning) return;
            _fillJobHandle.Complete();
            NativeArray<SpatialHashMapEntry> hashArray = _entryNativeList.AsArray();
            Entries = new SpatialHashMapEntry[hashArray.Length];
            hashArray.CopyTo(Entries);
            _fillJob.Spheres.Dispose();
            _entryNativeList.Dispose();
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
        private struct SortJob : IJob
        {
            public NativeArray<SpatialHashMapEntry> HashMap;

            public void Execute()
            {
                HashMap.Sort();
            }
        }

        [BurstCompile]
        private struct SolveCollisionConflicts : IJobParallelFor
        {
            [ReadOnly] public NativeArray<float3> Displacements;
            public NativeArray<float3> Positions;

            public void Execute(int index)
            {
                Positions[index] += Displacements[index] * 1.05f;
            }
        }

        [BurstCompile]
        private struct AccessHashMap : IJobParallelFor
        {
            public NativeArray<float3> AccessPoints;
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
                float3 result = new Vector3();
                int hash = Hash3(Mathf.FloorToInt(AccessPoints[index].x / GridSize),
                    Mathf.FloorToInt(AccessPoints[index].y / GridSize),
                    Mathf.FloorToInt(AccessPoints[index].z / GridSize));

                int start = LowerBound(HashMap, hash);
                if (start == HashMap.Length || HashMap[start].Hash != hash)
                    return;

                int end = UpperBound(HashMap, hash);

                for (int i = start; i < end; i++)
                {
                    if (hash != HashMap[i].Hash) continue;
                    if (Vector3.Distance(AccessPoints[index], HashMap[i].Target.Position) >
                        HashMap[i].Target.Radius) continue;

                    float3 direction = AccessPoints[index] - HashMap[i].Target.Position;
                    float distance = math.length(direction);
                    direction = math.normalize(direction);

                    result = direction * (HashMap[i].Target.Radius - distance);
                }

                AccessPoints[index] += result;
            }
        }


        [BurstCompile]
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

        public int CompareTo(SpatialHashMapEntry other)
        {
            return Hash.CompareTo(other.Hash);
        }
    }
}