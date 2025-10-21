using System;
using System.Collections.Generic;
using myXpbd.Parallel_XPBD.Collisions;
using Unity.Collections;
using Unity.Mathematics;
using UnityEngine;

public class ElipsoidInteraction : MonoBehaviour
{
    public Ellipsoid Ellipsoid;
    public GameObject probe;
    public EllipsoidSpatialHash HashMap;

    void Start()
    {
        Ellipsoid = new Ellipsoid();
        HashMap = new EllipsoidSpatialHash();
    }

    void Update()
    {
        Ellipsoid = new Ellipsoid()
        {
            Position = transform.position,
            Rotation = transform.rotation,
            HalfAxis = transform.localScale / 2
        };
    }

    private void OnDrawGizmos()
    {
        if (probe == null || HashMap == null) return;

        float3 result = fastestExit_world_fast(probe.transform.position, Ellipsoid.Position,
            Ellipsoid.Rotation, Ellipsoid.HalfAxis);
        float length = SdEllipsoid(probe.transform.position, Ellipsoid.Position,
            Ellipsoid.Rotation, Ellipsoid.HalfAxis);

        DrawArrow.ForGizmo(probe.transform.position, result * length * -1);

        Ellipsoid[] arr = { Ellipsoid };
        HashMap.SaveGridPositionsParallel(new NativeArray<Ellipsoid>(arr, Allocator.TempJob), 1);


        for (int i = 0; i < HashMap.Entries.Length; i++)
        {
            Gizmos.color = Color.red;
            Gizmos.DrawWireCube(
                new Vector3(HashMap.Entries[i].Corner.x, HashMap.Entries[i].Corner.y, HashMap.Entries[i].Corner.z) +
                Vector3.one * 0.5f, Vector3.one);
        }
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