using Unity.Entities;
using Unity.Mathematics;
using UnityEngine;

public class PhysicsBody : MonoBehaviour
{
    [SerializeField]
    Vector3 boxColliderDimensions = Vector3.one;

    public Unity.Physics.CollisionResponsePolicy CollisionResponse;

    public enum MotionType { Dynamic, Static }

    public MotionType Motion;
    public Vector3 BoxColliderSize => new Vector3(
        boxColliderDimensions.x * transform.localScale.x,
        boxColliderDimensions.y * transform.localScale.y,
        boxColliderDimensions.z * transform.localScale.z);

    public float Mass = 1;
    public float GravityScale = 1;

    /// <summary>
    /// Linear velocity in world space.
    /// </summary>
    public Vector3 Velocity;
    
    /// <summary>
    /// Angular velocity in world space.
    /// </summary>
    public Vector3 AngularVelocity
    {
        get => CalculateWorldAngularVelocity(LocalAngularVelocity);
        set => LocalAngularVelocity = CalculateLocalAngularVelocity(value);
    }

    public Vector3 LocalAngularVelocity;

    public int Entity;
    public BlobAssetReference<Unity.Physics.Collider> BoxCollider;

    public event System.Action<PhysicsBody> OnTrigger;

    // TODO: Internal.
    public void OnTriggerEvent(PhysicsBody other)
    {
        OnTrigger?.Invoke(other);
    }

    private float3 CalculateLocalAngularVelocity(Vector3 worldAngularVelocity)
    {
        quaternion inertiaOrientationInWorldSpace = math.mul(transform.rotation, BoxCollider.Value.MassProperties.MassDistribution.Transform.rot);
        float3 angularVelocityInertiaSpace = math.rotate(math.inverse(inertiaOrientationInWorldSpace), worldAngularVelocity);

        return angularVelocityInertiaSpace;
    }

    private float3 CalculateWorldAngularVelocity(Vector3 localAngularVelocity)
    {
        quaternion inertiaOrientationInWorldSpace = math.mul(transform.rotation, BoxCollider.Value.MassProperties.MassDistribution.Transform.rot);
        return math.rotate(inertiaOrientationInWorldSpace, localAngularVelocity);
    }

    private void OnDrawGizmosSelected()
    {
        var matrix = Gizmos.matrix;
        Gizmos.matrix = Matrix4x4.TRS(transform.position, transform.rotation, Vector3.one);
        Gizmos.color = new Color(0.55f, 0.95f, 0.55f, 0.75f);
        Gizmos.DrawWireCube(Vector3.zero, BoxColliderSize);
        Gizmos.matrix = matrix;
    }
}
