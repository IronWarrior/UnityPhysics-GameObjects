using Unity.Mathematics;
using Unity.Physics;

public interface IPhysicsBody
{
    RigidBody GetRigidbody();
    void SetRigidbody(RigidBody rb);

    bool IsKinematic { get; }
    float Mass { get; }
    float GravityScale { get; }
    float3 Velocity { get; set; }
    float3 LocalAngularVelocity { get; set; }
}
