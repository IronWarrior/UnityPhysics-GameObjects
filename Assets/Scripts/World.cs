using Unity.Physics;
using Unity.Mathematics;
using Unity.Entities;
using System.Linq;
using System;
using System.Collections.Generic;

public class World : IDisposable
{
    private PhysicsWorld world;

    private readonly BlobAssetStore store = new BlobAssetStore();

    // TODO: Better data structure for faster remove, etc.
    private readonly List<PhysicsBody> bodies = new List<PhysicsBody>();

    // Start at entity 1, since an entity with index 0/version 0 is considered null.
    // Maybe better to increment version to 1 initially?
    private int currentEntityIndex = 1;

    public World()
    {
        // Initialize the world's capacity to all zeroes, as the capacity
        // is reset every Rebuild() call.
        world = new PhysicsWorld(0, 0, 0);
    }

    public void Dispose()
    {
        store.Dispose();
        world.Dispose();
    }

    public void AddPhysicsBody(PhysicsBody pb)
    {
        BoxGeometry geo = new BoxGeometry()
        {
            Size = pb.BoxColliderSize,
            Orientation = quaternion.identity,
            Center = float3.zero
        };

        var mat = Material.Default;
        mat.CollisionResponse = CollisionResponsePolicy.CollideRaiseCollisionEvents;
        var blob = BoxCollider.Create(geo, CollisionFilter.Default, mat);
        store.AddUniqueBlobAsset(ref blob);

        pb.BoxCollider = blob;
        pb.Entity = currentEntityIndex;
        currentEntityIndex++;

        bodies.Add(pb);
    }

    public void RemovePhysicsBody(PhysicsBody pb)
    {
        bodies.Remove(pb);
    }

    public void Step(float deltaTime, float3 gravity)
    {
        Rebuild(deltaTime, gravity);

        SimulationStepInput input = new SimulationStepInput()
        {
            World = world,
            TimeStep = deltaTime,
            Gravity = gravity,
            NumSolverIterations = 5,
            // Setting this to true causes the dynamic world resulting from
            // integrating velocities to be written back to the collision world,
            // which is used to set the game object transform positions.
            SynchronizeCollisionWorld = true
        };

        SimulationContext context = new SimulationContext();
        context.Reset(input);

        Simulation.StepImmediate(input, ref context);

        // Can retrieve collision/trigger events here prior to disposal.
        context.Dispose();

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
                pb.AngularVelocity = world.MotionVelocities[rbIndex].AngularVelocity;
            }
        }
    }

    private void Rebuild(float deltaTime, float3 gravity)
    {
        // TODO: Can keep track of count with Add/Remove instead of recounting each time.
        int dynamicCount = bodies.Count((b) => b.Motion == PhysicsBody.MotionType.Dynamic);
        int staticCount = bodies.Count((b) => b.Motion == PhysicsBody.MotionType.Static);

        // Reset() resizes array capacities.
        world.Reset(staticCount, dynamicCount, 0);

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
                    AngularVelocity = pb.AngularVelocity,
                    InverseInertia = inverseInteriaTensor,
                    InverseMass = inverseMass,
                    GravityFactor = 1,
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
        world.UpdateIndexMaps();

        // Prepare the world for collision detection. If this method is not called no
        // collisions will occur during physics step.
        world.CollisionWorld.BuildBroadphase(ref world, deltaTime, gravity);
    }
}
