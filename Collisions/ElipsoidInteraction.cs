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
    public EllipsoidMap HashMap;
    public XpbdMesh toSimulate;
    public bool rotate;
    public int gridsize;
    private float3 _lastPosition;

    void Start()
    {
        Ellipsoid = new Ellipsoid();
    }

    void Update()
    {
        if (rotate) transform.Rotate(new Vector3(0, 0, 0.5f));
        _lastPosition = Ellipsoid.Position;
        float3 vel = toSimulate.sweepVolume ? (float3)transform.position - _lastPosition : float3.zero;
        Ellipsoid = new Ellipsoid()
        {
            Position = transform.position,
            Rotation = transform.rotation,
            HalfAxis = (transform.localScale / 2) * 1.03f,
            Velocity = vel
        };
        Ellipsoid[] arr = { Ellipsoid };
        if (toSimulate.xpbd == null) return;
        toSimulate.xpbd.HashMapEllipsoids.AddEllipsoidsToQueue(arr);
    }

    private void OnDrawGizmos()
    {
        if (toSimulate.xpbd == null || toSimulate.xpbd.HashMapEllipsoids == null) return;

        Gizmos.color = Color.red;
        Gizmos.DrawSphere(Ellipsoid.Position, Ellipsoid.HalfAxis.x);
        Gizmos.color = Color.blue;
        Gizmos.DrawSphere(_lastPosition, Ellipsoid.HalfAxis.x);
        Gizmos.color = Color.white;

        for (int i = 0; i < toSimulate.xpbd.HashMapEllipsoids.Entries.Length; i++)
        {
            // Gizmos.color = Color.cadetBlue;
            EllipsoidHashMapEntry entry = toSimulate.xpbd.HashMapEllipsoids.Entries[i];
            
            Gizmos.DrawWireSphere(entry.Target.Position, entry.Target.HalfAxis.x);
/*Gizmos.color = Color.red;
Gizmos.DrawWireCube(
    new Vector3(toSimulate.xpbd.HashMapEllipsoids.Entries[i].Corner.x,
        toSimulate.xpbd.HashMapEllipsoids.Entries[i].Corner.y,
        toSimulate.xpbd.HashMapEllipsoids.Entries[i].Corner.z) +
    Vector3.one * 0.5f, Vector3.one);*/
        }
    }


    public float3 dirExitApprox_local(float3 p, float3 r)
    {
        return math.normalize(p / (r * r));
    }

    public float3 fastestExit_world_fast(float3 p, float3 center, Quaternion rotation, float3 r)
    {
        Quaternion q = math.normalize(rotation);
        Quaternion qi = math.conjugate(q);

        float3 pl = math.rotate(qi, p - center);
        float3 dirL = dirExitApprox_local(pl, r);
        return (math.rotate(q, dirL)) * -0.5f * SdEllipsoid(p, center, rotation, r);
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