using System;
using Parallel_XPBD.Collisions;
using Unity.Mathematics;
using UnityEngine;

public class InteractionSphere : MonoBehaviour
{
    public XpbdMesh xpbdMesh;
    private Sphere _sphere;

    private void Start()
    {
        _sphere = new Sphere();
    }

    void Update()
    {
        _sphere.Radius = 1.01f * (transform.localScale.x / 2);
        _sphere.Position = transform.position;
        // if (xpbdMesh.handleCollisions) xpbdMesh.xpbd.SpatialHashMap.EnterSpheres(new Sphere[] { _sphere }, xpbdMesh, 2);
    }
    
}