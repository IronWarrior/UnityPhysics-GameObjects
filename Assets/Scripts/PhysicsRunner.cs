using System.Diagnostics;
using Unity.Mathematics;
using UnityEngine;

public class PhysicsRunner : MonoBehaviour
{
    [SerializeField]
    float deltaTime = 0.01f;

    [SerializeField]
    float3 gravity = new float3(0, -9.8f, 0);

    [SerializeField]
    int solverIterations = 5;

    [SerializeField]
    public Mode currentMode;

    [SerializeField]
    bool multithreaded = true;

    public enum Mode { UPhysics, PhysX };

    public World World { get; private set; }
    public event System.Action<float> OnUpdate;

    private float accumulatedDeltaTime;

    private void Awake()
    {
        World = new World();
    }

    private void OnDestroy()
    {
        World.Dispose();
    }

    private void Update()
    {
        accumulatedDeltaTime += Time.deltaTime;

        while (accumulatedDeltaTime > deltaTime)
        {
            OnUpdate?.Invoke(deltaTime);

            if (currentMode == Mode.UPhysics)
            {
                var bodies = FindObjectsOfType<PhysicsBody>();

                World.Step(bodies, deltaTime, gravity, Mathf.Clamp(solverIterations, 1, int.MaxValue), multithreaded);
            }
            else
            {
                Stopwatch stopwatch = new();
                stopwatch.Start();

                Physics.Simulate(deltaTime);

                stopwatch.Stop();

                UnityEngine.Debug.Log($"PhysX Step: {stopwatch.Elapsed.TotalMilliseconds}ms");
            }

            accumulatedDeltaTime -= deltaTime;
        }
    }
}
