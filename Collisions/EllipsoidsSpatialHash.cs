using System;
using System.Collections.Generic;
using System.Runtime.CompilerServices;
using Parallel_XPBD.Collisions;
using Unity.Burst;
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;
using static Unity.Mathematics.math;
using float3x3 = Unity.Mathematics.float3x3;

namespace myXpbd.Parallel_XPBD.Collisions
{
    public class EllipsoidSpatialHash
    {
        public EllipsoidHashMapEntry[] Entries;
        public float GridSize;

        public void SaveGlobalEllipsoidInLocalSpace(NativeArray<Ellipsoid> ellipsoids, XpbdMesh toSimulate, float gridSize)
        {
            NativeArray<Ellipsoid> outEllipsoids = new NativeArray<Ellipsoid>(ellipsoids.Length, Allocator.TempJob);
            ToLocalEllipsoidsJob toLocalJob = new ToLocalEllipsoidsJob()
            {
                WorldToLocal = toSimulate.transform.localToWorldMatrix,
                InEllipsoids = ellipsoids,
                OutEllipsoids = outEllipsoids
            };
            toLocalJob.ScheduleParallel(ellipsoids.Length, 32,default).Complete();
            
        }

        public void SaveGridPositionsParallel(NativeArray<Ellipsoid> ellipsoids, float gridSize)
        {
            GridSize = gridSize;
            var myList = new NativeList<EllipsoidHashMapEntry>(50000, Allocator.TempJob);

            var fillJob = new FillEllipsoidHashMap()
            {
                Result = myList.AsParallelWriter(),
                GridSize = gridSize,
                Ellipsoids = ellipsoids
            };

            JobHandle fillHandle = fillJob.Schedule(ellipsoids.Length, 64);

            var sortJob = new SortJob
            {
                HashMap = myList.AsDeferredJobArray()
            };

            JobHandle sortHandle = sortJob.Schedule(fillHandle);

            sortHandle.Complete();
            var hashArray = myList.AsArray();

            Entries = new EllipsoidHashMapEntry[hashArray.Length];
            hashArray.CopyTo(Entries);
            myList.Dispose();
            ellipsoids.Dispose();
        }


        [BurstCompile]
        private struct SortJob : IJob
        {
            public NativeArray<EllipsoidHashMapEntry> HashMap;

            public void Execute()
            {
                HashMap.Sort();
            }
        }

        [BurstCompile]
        private struct FillEllipsoidHashMap : IJobParallelFor
        {
            public NativeList<EllipsoidHashMapEntry>.ParallelWriter Result;
            public float GridSize;

            [ReadOnly] public NativeArray<Ellipsoid> Ellipsoids;

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
                float3 extent = OrientedEllipsoidExtent(Ellipsoids[index].Rotation, Ellipsoids[index].HalfAxis);
                float3 worldMin = Ellipsoids[index].Position - extent;
                float3 worldMax = Ellipsoids[index].Position + extent;

                int3 iMin = (int3)math.floor(worldMin / GridSize);
                int3 iMax = (int3)math.floor(worldMax / GridSize);

                float rMax = math.cmax(Ellipsoids[index].HalfAxis);
                float rMax2 = rMax * rMax;
                float3x3 R = new float3x3(Ellipsoids[index].Rotation);
                float3x3 RT = math.transpose(R);
                float3 invRadii = 1f / Ellipsoids[index].HalfAxis;

                for (int z = iMin.z; z <= iMax.z; z++)
                for (int y = iMin.y; y <= iMax.y; y++)
                for (int x = iMin.x; x <= iMax.x; x++)
                {
                    int3 idx = new int3(x, y, z);

                    float3 cellMin = GridSize * (float3)idx;
                    float3 cellMax = cellMin + GridSize;

                    if (!SphereIntersectsAABB(Ellipsoids[index].Position, rMax2, cellMin, cellMax))
                        continue;

                    if (EllipsoidIntersectsAABB_Sampled(Ellipsoids[index].Position, RT, invRadii, cellMin, cellMax))
                    {
                        EllipsoidHashMapEntry entry = new EllipsoidHashMapEntry()
                        {
                            Hash = Hash3(idx.x, idx.y, idx.z),
                            Target = Ellipsoids[index],
                            Corner = float3(idx.x, idx.y, idx.z)
                        };
                        Result.AddNoResize(entry);
                    }
                }
            }


            private static float3 OrientedEllipsoidExtent(quaternion rot, float3 radii)
            {
                float3x3 R = new float3x3(rot);
                float3x3 AbsR = new float3x3(math.abs(R.c0), math.abs(R.c1), math.abs(R.c2));

                return new float3(
                    math.dot(AbsR.c0, radii),
                    math.dot(AbsR.c1, radii),
                    math.dot(AbsR.c2, radii)
                );
            }

            private static bool SphereIntersectsAABB(float3 center, float r2, float3 bmin, float3 bmax)
            {
                float3 q = math.clamp(center, bmin, bmax);
                float3 d = center - q;
                return math.dot(d, d) <= r2;
            }

            private static bool EllipsoidIntersectsAABB_Sampled(float3 center, float3x3 RT, float3 invRadii,
                float3 bmin, float3 bmax)
            {
                float3 mid = (bmin + bmax) * 0.5f;
                if (InsideUnitSphere(TransformToEllipsoidLocal(mid, center, RT, invRadii)))
                    return true;

                for (int i = 0; i < 8; i++)
                {
                    float3 p = new float3(
                        (i & 1) == 0 ? bmin.x : bmax.x,
                        (i & 2) == 0 ? bmin.y : bmax.y,
                        (i & 4) == 0 ? bmin.z : bmax.z
                    );
                    if (InsideUnitSphere(TransformToEllipsoidLocal(p, center, RT, invRadii)))
                        return true;
                }

                if (math.all(center >= bmin) && math.all(center <= bmax))
                    return true;

                return false;
            }

            private static float3 TransformToEllipsoidLocal(float3 pWorld, float3 center, float3x3 RT, float3 invRadii)
            {
                float3 v = math.mul(RT, (pWorld - center));
                return v * invRadii;
            }

            private static bool InsideUnitSphere(float3 u)
                => math.dot(u, u) <= 1f + 1e-6f;
        }
    }

    [BurstCompile]
    public struct ToLocalEllipsoidsJob : IJobFor
    {
        [ReadOnly] public NativeArray<Ellipsoid> InEllipsoids;
        public NativeArray<Ellipsoid> OutEllipsoids;

        [ReadOnly] public float4x4 WorldToLocal; // toSimulate.transform.worldToLocalMatrix

        public void Execute(int i)
        {
            var e = InEllipsoids[i];

            // 1) Center transformieren (inkl. Translation)
            float4 p4 = mul(WorldToLocal, new float4(e.Position, 1f));
            float3 centerLocal = p4.xyz;

            // 2) Linearer Anteil A aus WorldToLocal
            float3x3 A = new float3x3(WorldToLocal.c0.xyz, WorldToLocal.c1.xyz, WorldToLocal.c2.xyz);

            // 3) Welt-Rotationsmatrix R
            float3x3 Rw = new float3x3(e.Rotation);

            // 4) M' = A * Rw * diag(r)
            float3 r = max(e.HalfAxis, new float3(0f)); // radii >= 0
            float3x3 B = mul(A, Rw);
            float3x3 Mprime = new float3x3(B.c0 * r.x, B.c1 * r.y, B.c2 * r.z);

            // 5) C = M'^T M'  (symmetrisch, positiv definit)
            float3x3 Mt = transpose(Mprime);
            float3x3 C = mul(Mt, Mprime);

            // 6) Eigenzerlegung von C: C = V * diag(s^2) * V^T
            float3 eval; // s^2
            float3x3 V; // orthonormal
            SymmetricEigen3x3(C, out eval, out V);

            // 7) Singularwerte s = sqrt(max(s^2, eps))
            const float eps = 1e-12f;
            float3 s = sqrt(max(eval, new float3(eps)));

            // 8) U = M' * V * diag(1/s)   (Orthonormalisierung; korrigiere evtl. Reflektion)
            float3 invS = 1f / s;
            float3x3 VinvS = new float3x3(V.c0 * invS.x, V.c1 * invS.y, V.c2 * invS.z);
            float3x3 U = mul(Mprime, VinvS);

            // numerische Orthonormalisierung von U (Gram-Schmidt + Re-orthonorm)
            Orthonormalize(ref U);

            // ensure det(U) = +1 (keine Spiegelung)
            if (Determinant(U) < 0f)
            {
                // Spiegel letzte Achse
                U.c2 = -U.c2;
                s.z = -s.z; // hält M' = U*Sigma*V^T konsistent; Radii müssen positiv:
                s.z = abs(s.z);
            }

            // 9) Ausgabe: Rotation' = U, Radii' = s
            var rotLocal = new quaternion(U);
            OutEllipsoids[i] = new Ellipsoid
            {
                Position = centerLocal,
                Rotation = rotLocal,
                HalfAxis = s
            };
        }

        // --------- Hilfsroutinen (Burst-freundlich) ---------

        // Jacobi-Eigenzerlegung für 3x3 symmetrische Matrix (einige Sweeps genügen)
        private static void SymmetricEigen3x3(in float3x3 A, out float3 eval, out float3x3 V)
        {
            // Start: V = I, B = A
            V = float3x3.identity;
            float3x3 B = A;

            for (int iter = 0; iter < 8; iter++) // 6-8 Sweeps sind meist mehr als genug
            {
                JacobiRotate(ref B, ref V, 0, 1);
                JacobiRotate(ref B, ref V, 0, 2);
                JacobiRotate(ref B, ref V, 1, 2);
            }

            eval = new float3(B.c0.x, B.c1.y, B.c2.z);

            // Nach Größe sortieren (absteigend), V entsprechend permutieren
            SortEigen(ref eval, ref V);
        }

        // Führt eine Jacobi-Rotation auf Indizes (p,q) durch
        private static void JacobiIndices(int a, out int i, out int j)
        {
            // mapping: (0,1)->(x,y), (0,2)->(x,z), (1,2)->(y,z)
            if (a == 0)
            {
                i = 0;
                j = 1;
            }
            else if (a == 1)
            {
                i = 0;
                j = 2;
            }
            else
            {
                i = 1;
                j = 2;
            }
        }

        private static void JacobiRotate(ref float3x3 B, ref float3x3 V, int p, int q)
        {
            // liest symmetrische Einträge
            float App = Get(B, p, p);
            float Aqq = Get(B, q, q);
            float Apq = Get(B, p, q);

            if (abs(Apq) < 1e-12f) return;

            float tau = (Aqq - App) / (2f * Apq);
            float t = rsqrt(1f + tau * tau);
            t = (tau >= 0f) ? (1f / (tau + (1f / t))) : (1f / (tau - (1f / t)));
            float c = rsqrt(1f + t * t);
            float s = t * c;

            // B' = J^T B J  (nur betroffene Elemente updaten)
            for (int k = 0; k < 3; k++)
            {
                float Bpk = Get(B, p, k);
                float Bqk = Get(B, q, k);
                float Bp_k_new = c * Bpk - s * Bqk;
                float Bq_k_new = s * Bpk + c * Bqk;
                Set(ref B, p, k, Bp_k_new);
                Set(ref B, q, k, Bq_k_new);
            }

            for (int k = 0; k < 3; k++)
            {
                float Bkp = Get(B, k, p);
                float Bkq = Get(B, k, q);
                float B_kp_new = c * Bkp - s * Bkq;
                float B_kq_new = s * Bkp + c * Bkq;
                Set(ref B, k, p, B_kp_new);
                Set(ref B, k, q, B_kq_new);
            }

            // Orthonormale Basis V mitdrehen: V' = V J
            for (int k = 0; k < 3; k++)
            {
                float Vkp = Get(V, k, p);
                float Vkq = Get(V, k, q);
                float V_kp_new = c * Vkp - s * Vkq;
                float V_kq_new = s * Vkp + c * Vkq;
                Set(ref V, k, p, V_kp_new);
                Set(ref V, k, q, V_kq_new);
            }
        }

        private static float Get(in float3x3 M, int r, int c)
        {
            if (c == 0) return r == 0 ? M.c0.x : (r == 1 ? M.c0.y : M.c0.z);
            if (c == 1) return r == 0 ? M.c1.x : (r == 1 ? M.c1.y : M.c1.z);
            return r == 0 ? M.c2.x : (r == 1 ? M.c2.y : M.c2.z);
        }

        private static void Set(ref float3x3 M, int r, int c, float v)
        {
            if (c == 0)
            {
                if (r == 0) M.c0.x = v;
                else if (r == 1) M.c0.y = v;
                else M.c0.z = v;
            }
            else if (c == 1)
            {
                if (r == 0) M.c1.x = v;
                else if (r == 1) M.c1.y = v;
                else M.c1.z = v;
            }
            else
            {
                if (r == 0) M.c2.x = v;
                else if (r == 1) M.c2.y = v;
                else M.c2.z = v;
            }
        }

        private static void SortEigen(ref float3 eval, ref float3x3 V)
        {
            // sort desc by eval
            for (int a = 0; a < 3; a++)
            for (int b = a + 1; b < 3; b++)
            {
                if (eval[a] < eval[b])
                {
                    float t = eval[a];
                    eval[a] = eval[b];
                    eval[b] = t;
                    // swap columns a <-> b in V
                    float3 tmp = GetCol(V, a);
                    SetCol(ref V, a, GetCol(V, b));
                    SetCol(ref V, b, tmp);
                }
            }
        }

        private static float3 GetCol(in float3x3 M, int c) => c == 0 ? M.c0 : (c == 1 ? M.c1 : M.c2);

        private static void SetCol(ref float3x3 M, int c, float3 v)
        {
            if (c == 0) M.c0 = v;
            else if (c == 1) M.c1 = v;
            else M.c2 = v;
        }

        private static void Orthonormalize(ref float3x3 U)
        {
            // Gram-Schmidt auf Spaltenvektoren
            float3 u0 = normalize(U.c0);
            float3 u1 = U.c1 - u0 * dot(u0, U.c1);
            u1 = normalize(u1);
            float3 u2 = U.c2 - u0 * dot(u0, U.c2) - u1 * dot(u1, U.c2);
            u2 = normalize(u2);
            U = new float3x3(u0, u1, u2);
        }

        private static float Determinant(in float3x3 M)
        {
            return dot(M.c0, cross(M.c1, M.c2));
        }
    }

    public struct EllipsoidHashMapEntry : IComparable<EllipsoidHashMapEntry>
    {
        public int Hash;
        public Ellipsoid Target;
        public float3 Corner;

        public int CompareTo(EllipsoidHashMapEntry other)
        {
            return Hash.CompareTo(other.Hash);
        }
    }
}