using System;
using Unity.Collections;
using Unity.Mathematics;
using UnityEngine;
using UnityEngine.InputSystem;

namespace Parallel_XPBD.Collisions
{
    public class SphereCollisions : MonoBehaviour
    {
        public SpatialHashMap HashMap;
        private Sphere[] _spheresInLocalSpace;

        private void Start()
        {
            HashMap = new SpatialHashMap();
        }

        public void EnterSpheres(Sphere[] spheres, XpbdMesh toSimulate)
        {
            _spheresInLocalSpace = new Sphere[spheres.Length];
            for (int i = 0; i < spheres.Length; i++)
            {
                _spheresInLocalSpace[i] = new Sphere()
                {
                    Position = toSimulate.transform.InverseTransformPoint(spheres[i].Position),
                    Radius = spheres[i].Radius / transform.localScale.x,
                };
            }

            HashMap.SaveGridPositionsParallel(_spheresInLocalSpace, 2);
        }
        
        // public float3[] GetDisplacementParallel(float3[] positions)
        // {
        //     return HashMap.AccessHashMapParallel(positions);
        // }

        private void OnDrawGizmos()
        {
            if (_spheresInLocalSpace == null) return;
            for (int i = 0; i < _spheresInLocalSpace.Length; i++)
            {
                Gizmos.color = Color.blue;
                Gizmos.DrawSphere(_spheresInLocalSpace[i].Position, _spheresInLocalSpace[i].Radius);
            }
        }
    }
}