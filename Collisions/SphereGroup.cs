using System;
using Parallel_XPBD.Collisions;
using Unity.Burst;
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;
using UnityEngine;
using Random = UnityEngine.Random;

public class SphereGroupBurst_DataOnly : MonoBehaviour
{
    [Header("Control")]
    [Min(0)]
    [Tooltip("Wie viele Spheres aktiv sein sollen (live im Play Mode änderbar).")]
    public int activeSpheres = 20;

    [Header("Spawn Settings")]
    [Tooltip("X- & Y-Range der Startpositionen.\nX: [0, rangeX]\nY: [-10, -10+rangeY]\nZ initial -3.")]
    public float rangeX = 25f;
    public float rangeY = 25f;
    [Tooltip("Amplitude der Z-Sinusbewegung.")]
    public float zAmplitude = 4f;
    [Tooltip("Min/Max Start-Radius (vor kRadMul).")]
    public Vector2 radiusRange = new Vector2(1f, 2f);
    [Tooltip("Optionaler Seed für reproduzierbare Starts. <0 = kein Seed.")]
    public int randomSeed = -1;

    [Header("XPBD Target")]
    public XpbdMesh _toSimulate;

    private NativeArray<Sphere> _spheresNative;       
    private NativeArray<float3> _startPositions;      
    private NativeArray<float>  _phasePerSphere;    
    private NativeArray<float>  _radiusPerSphere;    

    private const float kRadMul = 0.55f; 

    [BurstCompile]
    private struct MoveAndFillJob : IJobParallelFor
    {
        public NativeArray<Sphere> Spheres;            
        [ReadOnly] public NativeArray<float3> Starts; 
        [ReadOnly] public NativeArray<float> Phases;  
        [ReadOnly] public NativeArray<float> Radii;   

        public float TimeNow;   
        public float Amp;     

        public void Execute(int index)
        {
            float offset = math.sin(TimeNow + Phases[index]);
            float3 pos = Starts[index] + new float3(0f, 0f, offset * Amp);

            Spheres[index] = new Sphere
            {
                Position = pos,
                Radius   = Radii[index] * kRadMul
            };
        }
    }

    private void OnEnable()
    {
        if (randomSeed >= 0) Random.InitState(randomSeed);
        AllocateAndInit(activeSpheres, keepExisting: false);
    }

    private void OnDisable()
    {
        DisposeNatives();
    }

    private void OnValidate()
    {
        if (!Application.isPlaying) return;

        if (radiusRange.x > radiusRange.y)
            radiusRange = new Vector2(radiusRange.y, radiusRange.x);

        ResizeActiveCount(activeSpheres);
    }

    private void Update()
    {
        if (!_spheresNative.IsCreated || _spheresNative.Length == 0) return;
        if (_toSimulate == null || _toSimulate.xpbd == null) return;

        var job = new MoveAndFillJob
        {
            Spheres = _spheresNative,
            Starts  = _startPositions,
            Phases  = _phasePerSphere,
            Radii   = _radiusPerSphere,
            TimeNow = Time.time,
            Amp     = zAmplitude
        };

        _toSimulate.xpbd.TimeLogger.StartCollisionEntryClockwatch();
        var handle = job.Schedule(_spheresNative.Length, 64); // batch size 64
        handle.Complete();

        _toSimulate.xpbd.SpatialHashMap.EnterSpheres(_spheresNative, _toSimulate, 3);
        _toSimulate.xpbd.TimeLogger.StopCollisionEntryClockwatch();
    }

    // ---------- Public API ----------
    /// <summary>Per Code die Anzahl live setzen.</summary>
    public void SetActiveSpheres(int newCount)
    {
        newCount = Mathf.Max(0, newCount);
        activeSpheres = newCount;
        if (!Application.isPlaying) return;
        ResizeActiveCount(newCount);
    }

    // ---------- Internals ----------
    private void AllocateAndInit(int count, bool keepExisting)
    {
        // Alte Arrays ggf. übernehmen
        NativeArray<float3> oldStarts   = default;
        NativeArray<float>  oldPhases   = default;
        NativeArray<float>  oldRadii    = default;

        int oldLen = 0;
        if (keepExisting && _startPositions.IsCreated && _phasePerSphere.IsCreated && _radiusPerSphere.IsCreated)
        {
            oldLen = math.min(count,
                     math.min(_startPositions.Length,
                     math.min(_phasePerSphere.Length, _radiusPerSphere.Length)));
            if (oldLen > 0)
            {
                oldStarts = new NativeArray<float3>(oldLen, Allocator.Temp, NativeArrayOptions.UninitializedMemory);
                oldPhases = new NativeArray<float>(oldLen, Allocator.Temp, NativeArrayOptions.UninitializedMemory);
                oldRadii  = new NativeArray<float>(oldLen, Allocator.Temp, NativeArrayOptions.UninitializedMemory);
                NativeArray<float3>.Copy(_startPositions, oldStarts, oldLen);
                NativeArray<float>.Copy (_phasePerSphere, oldPhases, oldLen);
                NativeArray<float>.Copy (_radiusPerSphere, oldRadii,  oldLen);
            }
        }

        DisposeNatives();

        _spheresNative  = new NativeArray<Sphere>(count, Allocator.Persistent, NativeArrayOptions.UninitializedMemory);
        _startPositions = new NativeArray<float3>(count, Allocator.Persistent, NativeArrayOptions.UninitializedMemory);
        _phasePerSphere = new NativeArray<float>(count, Allocator.Persistent, NativeArrayOptions.UninitializedMemory);
        _radiusPerSphere= new NativeArray<float>(count, Allocator.Persistent, NativeArrayOptions.UninitializedMemory);

        int i = 0;

        // Übernommene Einträge zurückkopieren
        for (; i < oldLen; i++)
        {
            _startPositions[i] = oldStarts[i];
            _phasePerSphere[i] = oldPhases[i];
            _radiusPerSphere[i]= oldRadii[i];
        }

        // Neue Einträge initialisieren
        for (; i < count; i++)
        {
            float3 start = new float3(Random.value * rangeX,
                                      -10f + Random.value * rangeY,
                                      -3f);
            float baseRadius = Mathf.Lerp(radiusRange.x, radiusRange.y, Random.value);
            float phase = Random.value * 10f;

            _startPositions[i] = start;
            _phasePerSphere[i] = phase;
            _radiusPerSphere[i]= baseRadius;
        }

        // Temp kopien freigeben
        if (oldStarts.IsCreated) oldStarts.Dispose();
        if (oldPhases.IsCreated) oldPhases.Dispose();
        if (oldRadii.IsCreated)  oldRadii.Dispose();
    }

    private void ResizeActiveCount(int newCount)
    {
        newCount = Mathf.Max(0, newCount);
        int current = _spheresNative.IsCreated ? _spheresNative.Length : 0;
        if (newCount == current) return;

        // Reallokation, vorhandene Daten (vorn) behalten
        AllocateAndInit(newCount, keepExisting: true);
    }

    private void DisposeNatives()
    {
        if (_spheresNative.IsCreated)  _spheresNative.Dispose();
        if (_startPositions.IsCreated) _startPositions.Dispose();
        if (_phasePerSphere.IsCreated) _phasePerSphere.Dispose();
        if (_radiusPerSphere.IsCreated) _radiusPerSphere.Dispose();
    }
}
