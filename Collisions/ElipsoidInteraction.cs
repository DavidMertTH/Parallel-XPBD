using System;
using System.Collections.Generic;
using myXpbd.Parallel_XPBD.Collisions;
using Parallel_XPBD.Collisions;
using Unity.Collections;
using Unity.Mathematics;
using UnityEngine;
using UnityEngine.PlayerLoop;

public class ElipsoidInteraction : MonoBehaviour
{
    public Ellipsoid Ellipsoid;
    public GameObject probe;
    public EllipsoidSpatialHash HashMap;
    public XpbdMesh toSimulate;

    void Start()
    {
        Ellipsoid = new Ellipsoid();
    }

    void Update()
    {
        Ellipsoid = new Ellipsoid()
        {
            Position = transform.position,
            Rotation = transform.rotation,
            HalfAxis = transform.localScale / 2
        };
        Ellipsoid[] arr = { Ellipsoid };
        toSimulate.xpbd.HashMapEllipsoids.SaveGlobalEllipsoidInLocalSpace(new NativeArray<Ellipsoid>(arr, Allocator.TempJob), toSimulate, 5);
    }

    private void OnDrawGizmos()
    {
        // if ( toSimulate.xpbd.HashMapEllipsoids == null) return;
        //
        // for (int i = 0; i < toSimulate.xpbd.HashMapEllipsoids.Entries.Length; i++)
        // {
        //     Gizmos.color = Color.red;
        //     Gizmos.DrawWireCube(
        //         new Vector3(toSimulate.xpbd.HashMapEllipsoids.Entries[i].Corner.x, toSimulate.xpbd.HashMapEllipsoids.Entries[i].Corner.y, toSimulate.xpbd.HashMapEllipsoids.Entries[i].Corner.z) +
        //         Vector3.one * 0.5f, Vector3.one);
        // }
    }

    float3 dirExitApprox_local(float3 p, float3 r)
    {
        return math.normalize(p / (r * r));
    }

    float3 fastestExit_world_fast(float3 p, float3 center, Quaternion rotation, float3 r)
    {
        Quaternion q = math.normalize(rotation);
        Quaternion qi = math.conjugate(q);

        float3 pl = math.rotate(qi, p - center);
        float3 dirL = dirExitApprox_local(pl, r);
        return (math.rotate(q, dirL));
    }

    public static float SdEllipsoid(float3 p, float3 center, Quaternion rotation, float3 r)
    {
        float3 pl = math.mul(math.conjugate(rotation), p - center);

        float3 invR = 1f / r;
        float3 invR2 = invR * invR;

        float k0 = math.length(pl * invR);
        float k1 = math.length(pl * invR2);

        return k0 * (k0 - 1f) / k1;
    }
}