using System;
using Unity.Collections;
using Unity.Mathematics;
using UnityEngine;
using UnityEngine.InputSystem;

namespace Parallel_XPBD.Collisions
{
    public class SphereCollisions
    {
        public SpatialHashMap HashMap;
        private Sphere[] _spheresInLocalSpace;

        public SphereCollisions()
        {
            HashMap = new SpatialHashMap();
        }

       
    }
}