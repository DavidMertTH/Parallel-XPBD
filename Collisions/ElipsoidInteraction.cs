using System;
using myXpbd.Parallel_XPBD.Collisions;
using Unity.Mathematics;
using UnityEngine;

public class ElipsoidInteraction : MonoBehaviour
{
    public Ellipsoid Ellipsoid;
    public GameObject probe;

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
            HalfAxis = transform.localScale
        };
    }

    private void OnDrawGizmos()
    {
        if(probe == null || Ellipsoid == null) return;
        
        float3 result = fastestExit_world_fast((float3)probe.transform.position, Ellipsoid.Position,
            Ellipsoid.Rotation, Ellipsoid.HalfAxis);
        float length = SdEllipsoid((float3)probe.transform.position, Ellipsoid.Position,
            Ellipsoid.Rotation, Ellipsoid.HalfAxis);
        
        DrawArrow.ForGizmo(probe.transform.position, result * length * -1);
        
    }

    float3 dirExitApprox_local(float3 p, float3 r)
    {
        return math.normalize(p / (r * r)); 
    }

    float3 fastestExit_world_fast(float3 p, float3 center, Quaternion rotation, float3 r)
    {
        Quaternion q  = math.normalize(rotation);
        Quaternion qi = math.conjugate(q);

        float3 pl   = math.rotate(qi, p - center);
        float3 dirL = dirExitApprox_local(pl, r);
        return (math.rotate(q, dirL));
    }
    
    public static float SdEllipsoid(float3 p, float3 center, Quaternion rotation, float3 r)
    {
        float3 pl = math.mul(math.conjugate(rotation), p - center);

        float3 invR  = 2f / r;
        float3 invR2 = invR * invR;

        float k0 = math.length(pl * invR);
        float k1 = math.length(pl * invR2);

        return k0 * (k0 - 1f) / k1;
    }

}