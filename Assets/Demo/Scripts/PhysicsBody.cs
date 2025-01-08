using Unity.Entities;
using Unity.Mathematics;
using Unity.Physics;
using Unity.Physics.GameObjects;
using UnityEngine;

public class PhysicsBody : MonoBehaviour, IPhysicsBody
{
    [SerializeField] CollisionResponsePolicy collisionResponse = CollisionResponsePolicy.Collide;
    [SerializeField] bool isKinematic;
    [SerializeField] float mass = 1;
    [SerializeField] float gravityScale = 1;

    /// <summary>
    /// Linear velocity in world space.
    /// </summary>
    public float3 Velocity { get; set; }
    public float3 LocalAngularVelocity { get; set; }

    public bool IsKinematic => isKinematic;
    public float Mass => mass;
    public float GravityScale => gravityScale;

    /// <summary>
    /// Angular velocity in world space.
    /// </summary>
    public float3 AngularVelocity
    {
        get => IPhysicsBody.CalculateWorldAngularVelocity(LocalAngularVelocity, transform.rotation, physicsCollider);
        set => LocalAngularVelocity = IPhysicsBody.CalculateLocalAngularVelocity(value, transform.rotation, physicsCollider);
    }

    private BlobAssetReference<Unity.Physics.Collider> physicsCollider;

    public event System.Action<PhysicsBody> OnTrigger;

    private void Start()
    {
        physicsCollider = FindObjectOfType<PhysicsRunner>().World.GetColliderAsset(GetComponent<UnityEngine.Collider>(), collisionResponse);
    }

    public void OnTriggerEvent(PhysicsBody other)
    {
        OnTrigger?.Invoke(other);
    }

    public RigidBody GetRigidbody()
    {
        var rigidTransform = RigidTransform.identity;
        rigidTransform.pos = transform.position;
        rigidTransform.rot = transform.rotation;

        return new()
        {
            WorldFromBody = rigidTransform,
            Collider = physicsCollider,
            Scale = 1
        };
    }

    public void SetRigidbody(RigidBody rb)
    {
        transform.SetPositionAndRotation(rb.WorldFromBody.pos, rb.WorldFromBody.rot);
    }
}
