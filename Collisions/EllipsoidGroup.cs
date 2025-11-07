using System;
using myXpbd.Parallel_XPBD.Collisions; // <— dein Ellipsoid-Struct (Rotation sollte 'quaternion' sein)
using Unity.Burst;
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;
using UnityEngine;
using Random = UnityEngine.Random;

public class EllipsoidGroup : MonoBehaviour
{
    [Header("Control")]
    [Min(0)]
    [Tooltip("Wie viele Ellipsoide aktiv sein sollen (live im Play Mode änderbar).")]
    public int activeEllipsoids = 20;

    [Header("Spawn Settings")]
    [Tooltip("X- & Y-Range der Startpositionen.\nX: [0, rangeX]\nY: [-10, -10+rangeY]\nZ initial -3.")]
    public float rangeX = 25f;
    public float rangeY = 25f;

    [Tooltip("Amplitude der Z-Sinusbewegung.")]
    public float zAmplitude = 4f;

    [Tooltip("Min/Max Halbachsen-Längen (werden pro Achse uniform in diesem Intervall gezogen).")]
    public Vector2 halfAxisRange = new Vector2(1f, 2f);

    [Tooltip("Optionaler Seed für reproduzierbare Starts. <0 = kein Seed.")]
    public int randomSeed = -1;

    [Header("XPBD Target")]
    public XpbdMesh _toSimulate;

    // --------- Native Daten ----------
    private NativeArray<Ellipsoid> _ellipsoidsNative; // enthält Position/Rotation/HalfAxis
    private NativeArray<float3>    _startPositions;
    private NativeArray<float>     _phasePerEllipsoid;
    private NativeArray<float3>    _halfAxes;
    private NativeArray<quaternion> _rotations;

    private const float kAxisMul = 0.55f; // analog zu kRadMul bei Spheres

    // --------- Job ----------
    [BurstCompile]
    private struct MoveAndFillJob : IJobParallelFor
    {
        public NativeArray<Ellipsoid> Ellipsoids;      // output

        [ReadOnly] public NativeArray<float3> Starts;  // start pos
        [ReadOnly] public NativeArray<float>  Phases;  // zeitversatz
        [ReadOnly] public NativeArray<float3> HalfAxes;
        [ReadOnly] public NativeArray<quaternion> Rotations;

        public float TimeNow;
        public float Amp;

        public void Execute(int index)
        {
            float offset = math.sin(TimeNow + Phases[index]);
            float3 pos = Starts[index] + new float3(0f, 0f, offset * Amp);

            Ellipsoids[index] = new Ellipsoid
            {
                Position = pos,
                Rotation = Rotations[index],
                HalfAxis = HalfAxes[index] * kAxisMul
            };
        }
    }

    private void OnDrawGizmosSelected()
    {
        foreach (var ellipsoid in _ellipsoidsNative)
        {
            Gizmos.DrawWireSphere(ellipsoid.Position, 0.1f);
        }
    }

    // --------- Unity Lifecycle ----------
    private void OnEnable()
    {
        if (randomSeed >= 0) Random.InitState(randomSeed);
        AllocateAndInit(activeEllipsoids, keepExisting: false);
    }

    private void OnDestroy()
    {
        DisposeNatives();
    }

    private void OnValidate()
    {
        if (!Application.isPlaying) return;

        if (halfAxisRange.x > halfAxisRange.y)
            halfAxisRange = new Vector2(halfAxisRange.y, halfAxisRange.x);

        ResizeActiveCount(activeEllipsoids);
    }

    private void Update()
    {
        if (!_ellipsoidsNative.IsCreated || _ellipsoidsNative.Length == 0) return;
        if (_toSimulate == null || _toSimulate.xpbd == null) return;

        var job = new MoveAndFillJob
        {
            Ellipsoids = _ellipsoidsNative,
            Starts     = _startPositions,
            Phases     = _phasePerEllipsoid,
            HalfAxes   = _halfAxes,
            Rotations  = _rotations,
            TimeNow    = Time.time,
            Amp        = zAmplitude
        };

        _toSimulate.xpbd.TimeLogger.StartCollisionEntryClockwatch();
        var handle = job.Schedule(_ellipsoidsNative.Length, 64);
        handle.Complete();

        // ⬇️ Trage die Ellipsoide in deine XPBD-Struktur ein
        // Falls du eine passende Methode hast, z. B. EnterEllipsoids(...):
        Ellipsoid[] arr = new Ellipsoid[_ellipsoidsNative.Length];
        _ellipsoidsNative.CopyTo(arr);
        _toSimulate.xpbd.HashMapEllipsoids.AddEllipsoidsToQueue(arr);

        // Platzhalter: Wenn du (noch) keine EnterEllipsoids hast, baue sie analog zu EnterSpheres.
        _toSimulate.xpbd.TimeLogger.StopCollisionEntryClockwatch();
    }


    // --------- Internals ----------
    private void AllocateAndInit(int count, bool keepExisting)
    {
        // Alte Werte (teilweise) übernehmen
        NativeArray<float3>     oldStarts = default;
        NativeArray<float>      oldPhases = default;
        NativeArray<float3>     oldAxes   = default;
        NativeArray<quaternion> oldRots   = default;

        int oldLen = 0;
        if (keepExisting &&
            _startPositions.IsCreated && _phasePerEllipsoid.IsCreated &&
            _halfAxes.IsCreated && _rotations.IsCreated)
        {
            oldLen = math.min(count,
                     math.min(_startPositions.Length,
                     math.min(_phasePerEllipsoid.Length,
                     math.min(_halfAxes.Length, _rotations.Length))));
            if (oldLen > 0)
            {
                oldStarts = new NativeArray<float3>(oldLen, Allocator.Temp, NativeArrayOptions.UninitializedMemory);
                oldPhases = new NativeArray<float>(oldLen, Allocator.Temp, NativeArrayOptions.UninitializedMemory);
                oldAxes   = new NativeArray<float3>(oldLen, Allocator.Temp, NativeArrayOptions.UninitializedMemory);
                oldRots   = new NativeArray<quaternion>(oldLen, Allocator.Temp, NativeArrayOptions.UninitializedMemory);

                NativeArray<float3>.Copy(_startPositions, oldStarts, oldLen);
                NativeArray<float>.Copy (_phasePerEllipsoid, oldPhases, oldLen);
                NativeArray<float3>.Copy (_halfAxes, oldAxes, oldLen);
                NativeArray<quaternion>.Copy(_rotations, oldRots, oldLen);
            }
        }

        DisposeNatives();

        _ellipsoidsNative   = new NativeArray<Ellipsoid>(count, Allocator.Persistent, NativeArrayOptions.UninitializedMemory);
        _startPositions     = new NativeArray<float3>(count, Allocator.Persistent, NativeArrayOptions.UninitializedMemory);
        _phasePerEllipsoid  = new NativeArray<float>(count, Allocator.Persistent, NativeArrayOptions.UninitializedMemory);
        _halfAxes           = new NativeArray<float3>(count, Allocator.Persistent, NativeArrayOptions.UninitializedMemory);
        _rotations          = new NativeArray<quaternion>(count, Allocator.Persistent, NativeArrayOptions.UninitializedMemory);

        int i = 0;
        // Übernommene Einträge zurückkopieren
        for (; i < oldLen; i++)
        {
            _startPositions[i]    = oldStarts[i];
            _phasePerEllipsoid[i] = oldPhases[i];
            _halfAxes[i]          = oldAxes[i];
            _rotations[i]         = oldRots[i];
        }

        // Neue Einträge initialisieren
        for (; i < count; i++)
        {
            float3 start = new float3(Random.value * rangeX,
                                      -10f + Random.value * rangeY,
                                      -3f);

            // Halbachsen: uniform im Intervall und leichte Anisotropie
            float baseR = Mathf.Lerp(halfAxisRange.x, halfAxisRange.y, Random.value);
            // kleine zufällige Abweichung je Achse
            float3 jitter = new float3(
                math.lerp(0.85f, 1.15f, Random.value),
                math.lerp(0.85f, 1.15f, Random.value),
                math.lerp(0.85f, 1.15f, Random.value)
            );
            float3 axes = baseR * jitter;

            // Zufallsrotation (gleichverteilte Orientierung)
            quaternion rot = quaternion.identity;
            

            _startPositions[i]    = start;
            _phasePerEllipsoid[i] = Random.value * 10f;
            _halfAxes[i]          = math.max(new float3(1e-4f), axes);
            _rotations[i]         = rot;
        }

        if (oldStarts.IsCreated) oldStarts.Dispose();
        if (oldPhases.IsCreated) oldPhases.Dispose();
        if (oldAxes.IsCreated)   oldAxes.Dispose();
        if (oldRots.IsCreated)   oldRots.Dispose();
    }

    private void ResizeActiveCount(int newCount)
    {
        newCount = Mathf.Max(0, newCount);
        int current = _ellipsoidsNative.IsCreated ? _ellipsoidsNative.Length : 0;
        if (newCount == current) return;

        AllocateAndInit(newCount, keepExisting: true);
    }

    private void DisposeNatives()
    {
        Debug.Log("DisposeNatives");
        if (_ellipsoidsNative.IsCreated)  _ellipsoidsNative.Dispose();
        if (_startPositions.IsCreated)    _startPositions.Dispose();
        if (_phasePerEllipsoid.IsCreated) _phasePerEllipsoid.Dispose();
        if (_halfAxes.IsCreated)          _halfAxes.Dispose();
        if (_rotations.IsCreated)         _rotations.Dispose();
    }
}
