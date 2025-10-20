using System;
using Parallel_XPBD.Collisions;
using Unity.Mathematics;
using UnityEngine;
using UnityEngine.Jobs;
using Unity.Collections;
using Unity.Jobs;
using Unity.Burst;
using Random = UnityEngine.Random;

public class SphereGroupBurst : MonoBehaviour
{
    private GameObject[] _sphereObjects;
    public int count;
    // Native Speicher
    private TransformAccessArray _taa;
    private NativeArray<Sphere> _spheresNative;
    private NativeArray<float3> _startPositionsNative;
    private NativeArray<float> _randomPerSphereNative;

    // Falls EnterSpheres (noch) kein NativeArray akzeptiert:
    private Sphere[] _spheresManagedCache;

    public XpbdMesh _toSimulate;

    [BurstCompile]
    private struct MoveAndFillJob : IJobParallelForTransform
    {
        public NativeArray<Sphere> Spheres;              // out
        [ReadOnly] public NativeArray<float3> Starts;    // in
        [ReadOnly] public NativeArray<float> Randoms;    // in

        public float TimeNow;     // Time.time
        public float Amp;         // 4.0f
        private const float kRadMul = 0.55f; // 1.1f * 0.5f

        public void Execute(int index, TransformAccess transform)
        {
            // Bewegung (z-Achse)
            float offset = math.sin(TimeNow + Randoms[index]);
            float3 targetPos = Starts[index] + new float3(0f, 0f, offset * Amp);
            transform.position = targetPos;

            // Radius aus localScale.x berechnen
            float radius = kRadMul * transform.localScale.x;

            // Parallel das Kollisionsobjekt befüllen
            Spheres[index] = new Sphere
            {
                Position = targetPos,
                Radius = radius
            };
        }
    }
    
    private void OnEnable()
    {
        _sphereObjects = new GameObject[count];
        var startsManaged = new Vector3[count];
        var randomManaged = new float[count];

        for (int i = 0; i < count; i++)
        {
            var go = GameObject.CreatePrimitive(PrimitiveType.Sphere);
            go.transform.parent = transform;

            go.transform.position = new Vector3(Random.value * 25f, -10f + Random.value * 25f, -3f);
            float radius = 1f + Random.value;
            go.transform.localScale = new Vector3(radius, radius, radius);

            _sphereObjects[i]   = go;
            startsManaged[i]    = go.transform.position;
            randomManaged[i]    = Random.value * 10f;
        }

        // Persistent native Speicher vorbereiten
        _taa = new TransformAccessArray(Array.ConvertAll(_sphereObjects, s => s.transform));
        _spheresNative          = new NativeArray<Sphere>(count, Allocator.Persistent, NativeArrayOptions.UninitializedMemory);
        _startPositionsNative   = new NativeArray<float3>(count, Allocator.Persistent, NativeArrayOptions.UninitializedMemory);
        _randomPerSphereNative  = new NativeArray<float>(count, Allocator.Persistent, NativeArrayOptions.UninitializedMemory);

        // Einmalige Kopie der Startwerte
        for (int i = 0; i < count; i++)
        {
            _startPositionsNative[i]  = startsManaged[i];
            _randomPerSphereNative[i] = randomManaged[i];
        }

        _spheresManagedCache = new Sphere[count];
    }

    private void OnDisable()
    {
        if (_taa.isCreated) _taa.Dispose();
        if (_spheresNative.IsCreated) _spheresNative.Dispose();
        if (_startPositionsNative.IsCreated) _startPositionsNative.Dispose();
        if (_randomPerSphereNative.IsCreated) _randomPerSphereNative.Dispose();
    }

    private void Update()
    {
        var job = new MoveAndFillJob
        {
            Spheres = _spheresNative,
            Starts  = _startPositionsNative,
            Randoms = _randomPerSphereNative,
            TimeNow = Time.time,
            Amp     = 4f
        };

        _toSimulate.xpbd.TimeLogger.StartCollisionEntryClockwatch();
        var handle = job.Schedule(_taa);
        handle.Complete(); 
        
        _toSimulate.xpbd.SpatialHashMap.EnterSpheres(_spheresNative, _toSimulate, 3);

        _toSimulate.xpbd.TimeLogger.StopCollisionEntryClockwatch();
    }

    public void Rebuild(Transform[] newSphereTransforms)
    {
        // Dispose alt
        if (_taa.isCreated) _taa.Dispose();
        if (_spheresNative.IsCreated) _spheresNative.Dispose();
        if (_startPositionsNative.IsCreated) _startPositionsNative.Dispose();
        if (_randomPerSphereNative.IsCreated) _randomPerSphereNative.Dispose();

        // Neu aufsetzen
        int count = newSphereTransforms?.Length ?? 0;
        _taa = new TransformAccessArray(newSphereTransforms ?? Array.Empty<Transform>());
        _spheresNative         = new NativeArray<Sphere>(count, Allocator.Persistent, NativeArrayOptions.UninitializedMemory);
        _startPositionsNative  = new NativeArray<float3>(count, Allocator.Persistent, NativeArrayOptions.UninitializedMemory);
        _randomPerSphereNative = new NativeArray<float>(count, Allocator.Persistent, NativeArrayOptions.UninitializedMemory);
        _spheresManagedCache   = new Sphere[count];

        // Hier ggf. Starts/Randoms neu setzen
        for (int i = 0; i < count; i++)
        {
            _startPositionsNative[i]  = newSphereTransforms[i].position;
            _randomPerSphereNative[i] = UnityEngine.Random.value * 10f;
        }
    }
}