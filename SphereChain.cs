using System;
using System.Collections.Generic;
using System.Linq;
using Unity.Mathematics;
using UnityEngine;

public class SphereChain : MonoBehaviour
{
    public GameObject startPoint;
    public GameObject endPoint;
    public List<Sphere> Spheres;
    public float speed = 1;
    public GameObject handleSphere;

    private void Start()
    {
        Spheres = new List<Sphere>();
        InvokeRepeating(nameof(spawn), 1f, 2f);
    }
    

    private void spawn()
    {
        Sphere sphere = new Sphere();
        sphere.Position = startPoint.transform.position;
        sphere.Radius = 1f;
        Spheres.Add(sphere);
    }
    private void Update()
    {
        Spheres.Clear();
        Spheres.Add(new Sphere()
        {
            Position = transform.position,
            Radius = transform.localScale.x/2
        });
        /*
        Vector3 direction = endPoint.transform.position - startPoint.transform.position;
        direction.Normalize();
        for (int i = 0; i < Spheres.Count; i++)
        {
            Sphere sphere = new Sphere();
            sphere.Position = Spheres[i].Position + (float3)direction * speed * Time.deltaTime;
            sphere.Radius = Spheres[i].Radius;
            Spheres[i] = sphere;
            
            if (Vector3.Magnitude((Vector3)sphere.Position - startPoint.transform.position) >
                Vector3.Magnitude(endPoint.transform.position - startPoint.transform.position))
            {
                Spheres.RemoveAt(i);
            }
        }*/
    }

    private void OnDrawGizmos()
    {
        if(Spheres == null) return;
        Gizmos.color = Color.forestGreen;
        foreach (var sphere in Spheres)
        {
            Gizmos.DrawWireSphere(sphere.Position, sphere.Radius);
        }
    }
}