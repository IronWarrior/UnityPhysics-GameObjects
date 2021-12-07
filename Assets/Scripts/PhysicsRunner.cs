using Unity.Mathematics;
using UnityEngine;

public class PhysicsRunner : MonoBehaviour
{
    [SerializeField]
    float deltaTime = 0.01f;

    [SerializeField]
    float3 gravity = new float3(0, -9.8f, 0);

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
            world.Step(deltaTime, gravity);

            accumulatedDeltaTime -= deltaTime;
        }
    }
}
