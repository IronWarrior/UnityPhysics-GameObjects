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

    public event System.Action<float> OnUpdate;

    private World world;

    private float accumulatedDeltaTime;

    private void Awake()
    {
        world = new World();
    }

    private void Start()
    {
        foreach (var pb in FindObjectsOfType<PhysicsBody>())
        {
            world.AddPhysicsBody(pb);
        }

        foreach (var pj in FindObjectsOfType<PhysicsJoint>())
        {
            world.AddPhysicsJoint(pj);
        }
    }

    private void OnDestroy()
    {
        world.Dispose();
    }

    private void Update()
    {
        accumulatedDeltaTime += Time.deltaTime;

        while (accumulatedDeltaTime > deltaTime)
        {
            OnUpdate?.Invoke(deltaTime);

            world.Step(deltaTime, gravity, Mathf.Clamp(solverIterations, 1, int.MaxValue));

            accumulatedDeltaTime -= deltaTime;
        }
    }
}
