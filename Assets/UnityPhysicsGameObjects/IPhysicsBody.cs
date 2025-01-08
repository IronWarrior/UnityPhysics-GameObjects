using Unity.Entities;
using Unity.Mathematics;
using UnityEngine;

namespace Unity.Physics.GameObjects
{
    public interface IPhysicsBody
    {
        RigidBody GetRigidbody();
        void SetRigidbody(RigidBody rb);

        bool IsKinematic { get; }
        float Mass { get; }
        float GravityScale { get; }
        float3 Velocity { get; set; }
        float3 LocalAngularVelocity { get; set; }

        // Do people even do this?
        public static float3 CalculateLocalAngularVelocity(float3 worldAngularVelocity, quaternion rotation, BlobAssetReference<Collider> collider)
        {
            quaternion inertiaOrientationInWorldSpace = math.mul(rotation, collider.Value.MassProperties.MassDistribution.Transform.rot);
            float3 angularVelocityInertiaSpace = math.rotate(math.inverse(inertiaOrientationInWorldSpace), worldAngularVelocity);

            return angularVelocityInertiaSpace;
        }

        public static float3 CalculateWorldAngularVelocity(Vector3 localAngularVelocity, quaternion rotation, BlobAssetReference<Collider> collider)
        {
            quaternion inertiaOrientationInWorldSpace = math.mul(rotation, collider.Value.MassProperties.MassDistribution.Transform.rot);
            return math.rotate(inertiaOrientationInWorldSpace, localAngularVelocity);
        }
    }
}