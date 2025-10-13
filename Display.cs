using TMPro;
using UnityEngine;

public class Display : MonoBehaviour
{
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
    [HideInInspector] public float numParticles;
    [HideInInspector] public float numConstraints;


    void Update()
    {
        frames++;
        timer += Time.unscaledDeltaTime;

        CollisionTimeText.text = $"Collision: {collisionTime:F1} ms";
        SimTimeText.text = $"Simulation: {simTime:F1} ms";
        ParticleAmount.text = $"Particles: {numParticles:F1} ms";
        ConstraintAmount.text = $"Constraints: {numConstraints:F1} ms";
        CollisionSortTime.text = $"HashEntryTime: {colEntryTime:F1} ms";

        collisionTime = 0;
        simTime = 0;
        if (timer >= 0.5f)
        {
            float fpsTime = (float)(frames / timer);
            fps.text = $"{fpsTime:F1} FPS";
            frames = 0;
            timer = 0f;
        }
    }
}