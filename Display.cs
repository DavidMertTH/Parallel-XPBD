using TMPro;
using UnityEngine;

public class Display : MonoBehaviour
{
    [SerializeField] private XpbdMesh xpbdMesh;

    [SerializeField] private TMP_Text fps;
    [SerializeField] private TMP_Text SimTimeText;
    [SerializeField] private TMP_Text CollisionTimeText;
    [SerializeField] private TMP_Text CollisionSortTime;
    [SerializeField] private TMP_Text ParticleAmount;
    [SerializeField] private TMP_Text ConstraintAmount;

    private float timer;
    private int frames;

    [HideInInspector] public float collisionTime;
    [HideInInspector] public float simTime;
    [HideInInspector] public float colEntryTime;
    [HideInInspector] public int numParticles;
    [HideInInspector] public int numConstraints;


    void Update()
    {
        if (xpbdMesh == null) return;

        frames++;
        timer += Time.unscaledDeltaTime;

        if (timer <= 2f) return;


        numConstraints = xpbdMesh.Distances.Length;
        numParticles = xpbdMesh.Particles.Length;
        simTime = xpbdMesh.xpbd.TimeLogger._simTime;
        colEntryTime = xpbdMesh.xpbd.TimeLogger._colEntry;


        CollisionTimeText.text = $"Collision: {collisionTime:F1} ms";
        if(xpbdMesh.xpbd.TimeLogger.WasWaitingForThreadsToEnd) SimTimeText.text = $"Simulation: {simTime:F1} ms";
        else SimTimeText.text = $"Simulation:(<{simTime:F1}) ms";
        ParticleAmount.text = $"Particles: {numParticles:F1}";
        ConstraintAmount.text = $"Constraints: {numConstraints:F1}";
        CollisionSortTime.text = $"HashEntryTime: {colEntryTime:F1} ms";

        collisionTime = 0;
        simTime = 0;

        float fpsTime = (float)(frames / timer);
        fps.text = $"{fpsTime:F1} FPS";
        frames = 0;
        timer = 0f;
    }
}