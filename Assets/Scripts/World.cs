using Unity.Physics;
using Unity.Mathematics;
using Unity.Entities;
using System;
using System.Collections.Generic;

public class World : IDisposable
{
    private PhysicsWorld world;
    private SimulationContext context = new SimulationContext();

    private readonly BlobAssetStore store = new BlobAssetStore();

    // TODO: Better data structure for faster remove, etc.
    private readonly List<PhysicsBody> bodies = new List<PhysicsBody>();
    private readonly List<PhysicsJoint> physicsJoints = new List<PhysicsJoint>();

    // TODO: Dictionary iteration is non-deterministic in .NET...is there any issue with accessing individual elements?
    private readonly Dictionary<int, PhysicsBody> entityIdToBodies = new Dictionary<int, PhysicsBody>();

    // Start at entity 1, since an entity with index 0/version 0 is considered null.
    // Maybe better to increment version to 1 initially?
    private int currentEntityIndex = 1;

    private int dynamicCount, staticCount;

    public World()
    {
        // Initialize the world's capacity to all zeroes, as the capacity
        // is reset every Rebuild() call.
        world = new PhysicsWorld(0, 0, 0);
    }

    public void Dispose()
    {
        store.Dispose();
        context.Dispose();
        world.Dispose();
    }

    public void AddPhysicsJoint(PhysicsJoint joint)
    {
        physicsJoints.Add(joint);
    }

    public void RemovePhysicsJoint(PhysicsJoint joint)
    {
        physicsJoints.Remove(joint);
    }

    public void AddPhysicsBody(PhysicsBody pb)
    {
        if (pb.Motion == PhysicsBody.MotionType.Dynamic)
            dynamicCount++;
        else
            staticCount++;

        BoxGeometry geo = new BoxGeometry()
        {
            Size = pb.BoxColliderSize,
            Orientation = quaternion.identity,
            Center = float3.zero
        };

        var mat = Material.Default;
        mat.CollisionResponse = pb.CollisionResponse;
        var blob = BoxCollider.Create(geo, CollisionFilter.Default, mat);
        store.AddUniqueBlobAsset(ref blob);

        pb.BoxCollider = blob;
        pb.Entity = currentEntityIndex;
        currentEntityIndex++;

        bodies.Add(pb);
        entityIdToBodies[pb.Entity] = pb;
    }

    public void RemovePhysicsBody(PhysicsBody pb)
    {
        if (pb.Motion == PhysicsBody.MotionType.Dynamic)
            dynamicCount--;
        else
            staticCount--;

        bodies.Remove(pb);
        entityIdToBodies.Remove(pb.Entity);
    }

    public void Step(float deltaTime, float3 gravity, int solverIterations)
    {
        Rebuild(deltaTime, gravity);

        SimulationStepInput input = new SimulationStepInput()
        {
            World = world,
            TimeStep = deltaTime,
            Gravity = gravity,
            NumSolverIterations = solverIterations,
            // Setting this to true causes the dynamic world resulting from
            // integrating velocities to be written back to the collision world,
            // which is used to set the game object transform positions.
            SynchronizeCollisionWorld = true
        };

        context.Reset(input);

        Simulation.StepImmediate(input, ref context);

        // Copy the state from the PhysicsWorld back to the PhysicsBody components.
        foreach (var pb in bodies)
        {
            int rbIndex = world.GetRigidBodyIndex(new Entity() { Index = pb.Entity });
            var rb = world.Bodies[rbIndex];

            pb.transform.position = rb.WorldFromBody.pos;
            pb.transform.rotation = rb.WorldFromBody.rot;

            if (pb.Motion == PhysicsBody.MotionType.Dynamic)
            {
                pb.Velocity = world.MotionVelocities[rbIndex].LinearVelocity;
                pb.LocalAngularVelocity = world.MotionVelocities[rbIndex].AngularVelocity;
            }
        }

        var triggers = context.TriggerEvents.GetEnumerator();

        while (triggers.MoveNext())
        {
            var pbA = entityIdToBodies[world.Bodies[triggers.Current.BodyIndexA].Entity.Index];
            var pbB = entityIdToBodies[world.Bodies[triggers.Current.BodyIndexB].Entity.Index];

            pbA.OnTriggerEvent(pbB);
            pbB.OnTriggerEvent(pbA);
        }
    }

    private void Rebuild(float deltaTime, float3 gravity)
    {
        // Reset() resizes array capacities.
        world.Reset(staticCount, dynamicCount, physicsJoints.Count);

        // PhysicsWorld.DynamicBodies and .StaticBodies are sub arrays of the 
        // same native array, PhysicsWorld.Bodies.
        int staticIndex = 0, dynamicIndex = 0;

        var statics = world.StaticBodies;
        var dynamics = world.DynamicBodies;

        var motionDatas = world.MotionDatas;
        var motionVelocities = world.MotionVelocities;

        foreach (var pb in bodies)
        {
            var transform = RigidTransform.identity;
            transform.pos = pb.transform.position;
            transform.rot = pb.transform.rotation;

            RigidBody rb = new RigidBody()
            {
                WorldFromBody = transform,
                Entity = new Entity() { Index = pb.Entity },
                Collider = pb.BoxCollider
            };

            if (pb.Motion == PhysicsBody.MotionType.Dynamic)
            {
                dynamics[dynamicIndex] = rb;

                float inverseMass = math.rcp(pb.Mass);
                float3 inverseInteriaTensor = math.rcp(pb.BoxCollider.Value.MassProperties.MassDistribution.InertiaTensor * pb.Mass);
                float angularExpansionFactor = pb.BoxCollider.Value.MassProperties.AngularExpansionFactor;

                motionVelocities[dynamicIndex] = new MotionVelocity()
                {
                    LinearVelocity = pb.Velocity,
                    AngularVelocity = pb.LocalAngularVelocity,
                    InverseInertia = inverseInteriaTensor,
                    InverseMass = inverseMass,
                    GravityFactor = pb.GravityScale,
                    AngularExpansionFactor = angularExpansionFactor
                };

                quaternion interiaOrientation = pb.BoxCollider.Value.MassProperties.MassDistribution.Transform.rot;
                float3 centerOfMass = pb.BoxCollider.Value.MassProperties.MassDistribution.Transform.pos;

                motionDatas[dynamicIndex] = new MotionData()
                {
                    WorldFromMotion = new RigidTransform(
                        math.mul(pb.transform.rotation, interiaOrientation),
                        math.rotate(pb.transform.rotation, centerOfMass) + (float3)pb.transform.position),
                    BodyFromMotion = new RigidTransform(interiaOrientation, centerOfMass),
                    LinearDamping = 0,
                    AngularDamping = 0                    
                };

                dynamicIndex++;
            }
            else
            {
                statics[staticIndex] = rb;
                staticIndex++;
            }
        }

        // Updating index maps after rebuid is essential to be able to retrieve
        // rigidbodies by their entity ID, which is done when we want to copy
        // the rigidbody state back to the PhysicsBody component.
        world.CollisionWorld.UpdateBodyIndexMap();

        var joints = world.Joints;

        for (int i = 0; i < physicsJoints.Count; i++)
        {
            var physicsJoint = physicsJoints[i];

            int bodyIndexB = world.GetRigidBodyIndex(new Entity() { Index = physicsJoint.Body.Entity });
            int bodyIndexA = world.GetRigidBodyIndex(new Entity() { Index = physicsJoint.Target.Entity });

            joints[i] = new Joint()
            {
                Entity = new Entity() { Index = physicsJoint.Body.Entity },
                BodyPair = new BodyIndexPair() { BodyIndexA = bodyIndexA, BodyIndexB = bodyIndexB },
                EnableCollision = 0,
                AFromJoint = new Unity.Physics.Math.MTransform(quaternion.identity, float3.zero),
                BFromJoint = new Unity.Physics.Math.MTransform(quaternion.Euler(physicsJoint.AngularAnchor), physicsJoint.Anchor),
                Version = 0,
                Constraints = new Unity.Collections.FixedList128Bytes<Constraint>
                {
                    Length = 2,
                    [0] = Constraint.BallAndSocket(),
                    [1] = Constraint.FixedAngle()
                }
            };
        }

        world.DynamicsWorld.UpdateJointIndexMap();

        // Prepare the world for collision detection. If this method is not called no
        // collisions will occur during physics step.
        world.CollisionWorld.BuildBroadphase(ref world, deltaTime, gravity);
    }
}
