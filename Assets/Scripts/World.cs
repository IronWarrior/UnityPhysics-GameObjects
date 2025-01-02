using Unity.Physics;
using Unity.Mathematics;
using Unity.Entities;
using System;
using System.Collections.Generic;
using Unity.Collections;
using System.Diagnostics;

public class World : IDisposable
{
    public PhysicsWorld PhysicsWorld;

    //private SimulationContext context = new SimulationContext();

    private readonly BlobAssetStore store = new BlobAssetStore(1000);

    // TODO: Better data structure for faster remove, etc.
    private readonly List<PhysicsBody> bodies = new List<PhysicsBody>();
    private readonly List<PhysicsJoint> physicsJoints = new List<PhysicsJoint>();

    // TODO: Dictionary iteration is non-deterministic in .NET...is there any issue with accessing individual elements?
    private readonly Dictionary<int, PhysicsBody> entityIdToBodies = new Dictionary<int, PhysicsBody>();

    // Start at entity 1, since an entity with index 0/version 0 is considered null.
    // Maybe better to increment version to 1 initially?
    private int currentEntityIndex = 1;

    private int dynamicCount, staticCount;

    private Simulation sim = Simulation.Create();

    public World()
    {
        // Initialize the world's capacity to all zeroes, as the capacity
        // is reset every Rebuild() call.
        PhysicsWorld = new PhysicsWorld(0, 0, 0);
    }

    public void Dispose()
    {
        store.Dispose();
        //context.Dispose();
        PhysicsWorld.Dispose();
        sim.Dispose();
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
        store.TryAdd(ref blob);

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

    public void Step(float deltaTime, float3 gravity, int solverIterations, bool multithreaded)
    {
        int previousStaticBodyCount = PhysicsWorld.NumStaticBodies;

        BuildPhysicsWorld(deltaTime, gravity);

        SimulationStepInput input = new()
        {
            World = PhysicsWorld,
            TimeStep = deltaTime,
            Gravity = gravity,
            NumSolverIterations = solverIterations,
            // Setting this to true causes the dynamic world resulting from
            // integrating velocities to be written back to the collision world,
            // which is used to set the game object transform positions.
            SynchronizeCollisionWorld = true
        };

        Stopwatch stopwatch = new();

        // Prepare the world for collision detection. If this method is not called no
        // collisions will occur during physics step.
        if (multithreaded)
        {
            var shouldBuildTree = new NativeReference<int>(staticCount != previousStaticBodyCount ? 1 : 0, Allocator.TempJob);
            var buildHandle = PhysicsWorld.CollisionWorld.ScheduleBuildBroadphaseJobs(ref PhysicsWorld, deltaTime, gravity, shouldBuildTree, default);

            buildHandle.Complete();
            shouldBuildTree.Dispose();

            sim.ResetSimulationContext(input);

            stopwatch.Start();

            SimulationJobHandles handles = sim.ScheduleStepJobs(input, default, true);

            handles.FinalExecutionHandle.Complete();
            handles.FinalDisposeHandle.Complete();

            stopwatch.Stop();
        }
        else
        {
            PhysicsWorld.CollisionWorld.BuildBroadphase(ref PhysicsWorld, deltaTime, gravity);

            stopwatch.Start();

            sim.ResetSimulationContext(input);
            sim.Step(input);

            stopwatch.Stop();
        }

        UnityEngine.Debug.Log($"U Physics step: {stopwatch.Elapsed.TotalMilliseconds}ms");

        // Copy the state from the PhysicsWorld back to the PhysicsBody components.
        foreach (var pb in bodies)
        {
            int rbIndex = PhysicsWorld.GetRigidBodyIndex(new Entity() { Index = pb.Entity });
            var rb = PhysicsWorld.Bodies[rbIndex];

            pb.transform.position = rb.WorldFromBody.pos;
            pb.transform.rotation = rb.WorldFromBody.rot;

            if (pb.Motion == PhysicsBody.MotionType.Dynamic)
            {
                pb.Velocity = PhysicsWorld.MotionVelocities[rbIndex].LinearVelocity;
                pb.LocalAngularVelocity = PhysicsWorld.MotionVelocities[rbIndex].AngularVelocity;
            }
        }

        var triggers = sim.TriggerEvents.GetEnumerator();

        while (triggers.MoveNext())
        {
            var pbA = entityIdToBodies[PhysicsWorld.Bodies[triggers.Current.BodyIndexA].Entity.Index];
            var pbB = entityIdToBodies[PhysicsWorld.Bodies[triggers.Current.BodyIndexB].Entity.Index];

            pbA.OnTriggerEvent(pbB);
            pbB.OnTriggerEvent(pbA);
        }
    }

    private void BuildPhysicsWorld(float deltaTime, float3 gravity)
    {
        int previousStaticBodyCount = PhysicsWorld.NumStaticBodies;
        // Reset() resizes array capacities.
        PhysicsWorld.Reset(staticCount, dynamicCount, 0);

        // PhysicsWorld.DynamicBodies and .StaticBodies are sub arrays of the 
        // same native array, PhysicsWorld.Bodies.
        int staticIndex = 0, dynamicIndex = 0;

        var statics = PhysicsWorld.StaticBodies;
        var dynamics = PhysicsWorld.DynamicBodies;

        var motionDatas = PhysicsWorld.MotionDatas;
        var motionVelocities = PhysicsWorld.MotionVelocities;

        foreach (var pb in bodies)
        {
            var transform = RigidTransform.identity;
            transform.pos = pb.transform.position;
            transform.rot = pb.transform.rotation;

            RigidBody rb = new RigidBody()
            {
                WorldFromBody = transform,
                Entity = new Entity() { Index = pb.Entity },
                Collider = pb.BoxCollider,
                Scale = 1
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
        PhysicsWorld.CollisionWorld.UpdateBodyIndexMap();

        // Joint API was changed in 1.0, some properties are internal. Will
        // need to modify the code a bit to get them working again.

        //var joints = PhysicsWorld.Joints;

        //for (int i = 0; i < physicsJoints.Count; i++)
        //{
        //    var physicsJoint = physicsJoints[i];

        //    int bodyIndexB = PhysicsWorld.GetRigidBodyIndex(new Entity() { Index = physicsJoint.Body.Entity });
        //    int bodyIndexA = PhysicsWorld.GetRigidBodyIndex(new Entity() { Index = physicsJoint.Target.Entity });

        //    joints[i] = new Joint()
        //    {
        //        Entity = new Entity() { Index = physicsJoint.Body.Entity },
        //        BodyPair = new BodyIndexPair() { BodyIndexA = bodyIndexA, BodyIndexB = bodyIndexB },
        //        EnableCollision = 0,
        //        AFromJoint = new Unity.Physics.Math.MTransform(quaternion.identity, float3.zero),
        //        BFromJoint = new Unity.Physics.Math.MTransform(quaternion.Euler(physicsJoint.AngularAnchor), physicsJoint.Anchor),
        //        Version = 0,
        //        Constraints = new Unity.Collections.FixedList128Bytes<Constraint>
        //        {
        //            Length = 2,
        //            [0] = Constraint.BallAndSocket(),
        //            [1] = Constraint.FixedAngle()
        //        }
        //    };
        //}

        //PhysicsWorld.DynamicsWorld.UpdateJointIndexMap();
    }
}
