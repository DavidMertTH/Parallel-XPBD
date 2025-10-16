using System;
using Parallel_XPBD.Collisions;
using Unity.Mathematics;
using UnityEngine;
using Random = UnityEngine.Random;

public class SphereGroup : MonoBehaviour
{
    private Sphere[] _spheres;
    private GameObject[] _sphereObjects;
    private Vector3[] _startPositions;
    private float[] randomPerSphere;

    private SpatialHashMap[] _hashMap;
    public XpbdMesh _toSimulate;

    private void Start()
    {
        _sphereObjects = new GameObject[200];
        _startPositions = new Vector3[_sphereObjects.Length];
        randomPerSphere = new float[_sphereObjects.Length];
        for (int x = 0; x < _sphereObjects.Length; x++)
        {
            _sphereObjects[x] = GameObject.CreatePrimitive(PrimitiveType.Sphere);
            _sphereObjects[x].transform.position = new Vector3(Random.value * 10, -10 + Random.value * 10, -3);
            _sphereObjects[x].transform.parent = transform;
            float radius = 1 + Random.value;
            _sphereObjects[x].transform.localScale = new Vector3(radius, radius, radius);
            _startPositions[x] = _sphereObjects[x].transform.position;
            randomPerSphere[x] = Random.value * 10f;
        }
    }

    private void Update()
    {
        for (int i = 0; i < _sphereObjects.Length; i++)
        {
            float offset = Mathf.Sin(Time.time + randomPerSphere[i]); // oder realtimeSinceStartup
            _sphereObjects[i].transform.position = _startPositions[i] + Vector3.forward * (offset * 4);
        }
    }


    private void LateUpdate()
    {
        _spheres = new Sphere[_sphereObjects.Length];
        for (int i = 0; i < _sphereObjects.Length; i++)
        {
            _spheres[i] = new Sphere()
            {
                Position = _sphereObjects[i].transform.position,
                Radius = 1.06f * (_sphereObjects[i].transform.localScale.x / 2)
            };
        }

        _toSimulate.xpbd.TimeLogger.StartCollisionEntryClockwatch();
        _toSimulate.xpbd.SpatialHashMap.EnterSpheres(_spheres, _toSimulate, 3);
        _toSimulate.xpbd.TimeLogger.StopCollisionEntryClockwatch();
    }
}