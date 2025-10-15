using System;
using Parallel_XPBD.Collisions;
using UnityEngine;

public class InteractionSphere : MonoBehaviour
{
    public SphereCollisions sphereCollisions;
    public XpbdMesh xpbdMesh;

    private Sphere _sphere;

    private void Start()
    {
        _sphere = new Sphere();
    }

    void Update()
    {
        xpbdMesh.xpbd.SphereCollisions = sphereCollisions;

        _sphere.Radius = transform.localScale.x / 2;
        _sphere.Position = transform.position;
        if (xpbdMesh.handleCollisions) sphereCollisions.EnterSpheres(new Sphere[] { _sphere }, xpbdMesh);
    }
}