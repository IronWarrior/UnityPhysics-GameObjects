using Unity.Collections;
using Unity.Mathematics;
using UnityEngine;
using Unity.Physics;
using Unity.Physics.GameObjects;

public class DemoOverlap : MonoBehaviour
{
    [SerializeField]
    float3 halfExtents = new float3(0.5f, 0.5f, 0.5f);

    private void Update()
    {
        NativeList<DistanceHit> hits = new NativeList<DistanceHit>(Allocator.Temp);

        CollisionFilter filter = new CollisionFilter()
        {
            BelongsTo = ~0u,
            CollidesWith = ~0u,
        };

        FindObjectOfType<PhysicsRunner>().World.PhysicsWorld.OverlapBox(transform.position, transform.rotation, halfExtents, ref hits, filter, QueryInteraction.Default);

        foreach (var hit in hits)
        {
            Debug.Log(hit.Entity);
        }
    }
}
