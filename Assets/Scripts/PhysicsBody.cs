using Unity.Entities;
using Unity.Mathematics;
using Unity.Physics;
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
        get => CalculateWorldAngularVelocity(LocalAngularVelocity);
        set => LocalAngularVelocity = CalculateLocalAngularVelocity(value);
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

    private float3 CalculateLocalAngularVelocity(Vector3 worldAngularVelocity)
    {
        quaternion inertiaOrientationInWorldSpace = math.mul(transform.rotation, physicsCollider.Value.MassProperties.MassDistribution.Transform.rot);
        float3 angularVelocityInertiaSpace = math.rotate(math.inverse(inertiaOrientationInWorldSpace), worldAngularVelocity);

        return angularVelocityInertiaSpace;
    }

    private float3 CalculateWorldAngularVelocity(Vector3 localAngularVelocity)
    {
        quaternion inertiaOrientationInWorldSpace = math.mul(transform.rotation, physicsCollider.Value.MassProperties.MassDistribution.Transform.rot);
        return math.rotate(inertiaOrientationInWorldSpace, localAngularVelocity);
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
