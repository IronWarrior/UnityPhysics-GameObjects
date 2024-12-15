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
    Mode currentMode;

    public enum Mode { UPhysics, PhysX };

    public World World { get; private set; }
    public event System.Action<float> OnUpdate;

    private float accumulatedDeltaTime;

    private void Awake()
    {
        World = new World();
    }

    private void Start()
    {
        foreach (var pb in FindObjectsOfType<PhysicsBody>())
        {
            World.AddPhysicsBody(pb);
        }

        foreach (var pj in FindObjectsOfType<PhysicsJoint>())
        {
            World.AddPhysicsJoint(pj);
        }
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
                World.Step(deltaTime, gravity, Mathf.Clamp(solverIterations, 1, int.MaxValue));
            else
            {
                Stopwatch sw = new Stopwatch();

                sw.Start();

                Physics.Simulate(deltaTime);

                sw.Stop();

                UnityEngine.Debug.Log($"SW: {sw.Elapsed.TotalMilliseconds}");
            }

            accumulatedDeltaTime -= deltaTime;
        }
    }
}
