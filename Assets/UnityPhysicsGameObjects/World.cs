using Unity.Mathematics;
using Unity.Entities;
using System;
using System.Collections.Generic;
using Unity.Collections;
using System.Diagnostics;

namespace Unity.Physics.GameObjects
{
    public class World : IDisposable
    {
        public PhysicsWorld PhysicsWorld;

        private readonly BlobAssetStore store = new(1000);

        // TODO: Dictionary iteration is non-deterministic in .NET...is there any issue with accessing individual elements?
        private readonly Dictionary<int, PhysicsBody> entityIdToBodies = new();
        private readonly Simulation sim = Simulation.Create();

        private static readonly Stopwatch stopwatch = new();

        public World()
        {
            // Initialize the world's capacity to all zeroes, as the capacity
            // is reset every Rebuild() call.
            PhysicsWorld = new PhysicsWorld(0, 0, 0);
        }

        public void Dispose()
        {
            store.Dispose();
            PhysicsWorld.Dispose();
            sim.Dispose();
        }

        public void Step(IPhysicsBody[] physicsBodies, float deltaTime, float3 gravity, int solverIterations)
        {
            BuildPhysicsWorld(physicsBodies, out bool hasStaticCountChanged);

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

            stopwatch.Restart();

            var shouldBuildTree = new NativeReference<int>(hasStaticCountChanged ? 1 : 0, Allocator.TempJob);
            var buildHandle = PhysicsWorld.CollisionWorld.ScheduleBuildBroadphaseJobs(ref PhysicsWorld, deltaTime, gravity, shouldBuildTree, default);

            buildHandle.Complete();
            shouldBuildTree.Dispose();

            sim.ResetSimulationContext(input);

            SimulationJobHandles handles = sim.ScheduleStepJobs(input, default, true);

            handles.FinalExecutionHandle.Complete();
            handles.FinalDisposeHandle.Complete();

            stopwatch.Stop();

            UnityEngine.Debug.Log($"U Physics step: {stopwatch.Elapsed.TotalMilliseconds}ms");

            // Copy the state from the PhysicsWorld back to the PhysicsBody components.
            for (int i = 0; i < physicsBodies.Length; i++)
            {
                var pb = physicsBodies[i];

                int rbIndex = PhysicsWorld.GetRigidBodyIndex(new Entity() { Index = i + 1 });
                var rb = PhysicsWorld.Bodies[rbIndex];

                pb.SetRigidbody(rb);

                if (pb.IsKinematic == false)
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

        private void BuildPhysicsWorld(IPhysicsBody[] physicsBodies, out bool hasStaticCountChanged)
        {
            int staticCount = 0, dynamicCount = 0;

            foreach (var body in physicsBodies)
            {
                if (body.IsKinematic == true)
                    staticCount++;
                else
                    dynamicCount++;
            }

            hasStaticCountChanged = staticCount != PhysicsWorld.NumStaticBodies;

            // Reset() resizes array capacities.
            PhysicsWorld.Reset(staticCount, dynamicCount, 0);

            // PhysicsWorld.DynamicBodies and .StaticBodies are sub arrays of the 
            // same native array, PhysicsWorld.Bodies.
            int staticIndex = 0, dynamicIndex = 0;

            var statics = PhysicsWorld.StaticBodies;
            var dynamics = PhysicsWorld.DynamicBodies;

            var motionDatas = PhysicsWorld.MotionDatas;
            var motionVelocities = PhysicsWorld.MotionVelocities;

            for (var i = 0; i < physicsBodies.Length; i++)
            {
                var pb = physicsBodies[i];

                RigidBody rb = pb.GetRigidbody();

                rb.Entity = new() { Index = i + 1 };

                if (pb.IsKinematic == false)
                {
                    dynamics[dynamicIndex] = rb;

                    float inverseMass = math.rcp(pb.Mass);
                    float3 inverseInteriaTensor = math.rcp(rb.Collider.Value.MassProperties.MassDistribution.InertiaTensor * pb.Mass);
                    float angularExpansionFactor = rb.Collider.Value.MassProperties.AngularExpansionFactor;

                    motionVelocities[dynamicIndex] = new MotionVelocity()
                    {
                        LinearVelocity = pb.Velocity,
                        AngularVelocity = pb.LocalAngularVelocity,
                        InverseInertia = inverseInteriaTensor,
                        InverseMass = inverseMass,
                        GravityFactor = pb.GravityScale,
                        AngularExpansionFactor = angularExpansionFactor
                    };

                    quaternion interiaOrientation = rb.Collider.Value.MassProperties.MassDistribution.Transform.rot;
                    float3 centerOfMass = rb.Collider.Value.MassProperties.MassDistribution.Transform.pos;

                    motionDatas[dynamicIndex] = new MotionData()
                    {
                        WorldFromMotion = new RigidTransform(
                            math.mul(rb.WorldFromBody.rot, interiaOrientation),
                            math.rotate(rb.WorldFromBody.rot, centerOfMass) + rb.WorldFromBody.pos),
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

        public BlobAssetReference<Collider> GetColliderAsset(UnityEngine.Collider collider, CollisionResponsePolicy collisionResponse)
        {
            var mat = Material.Default;
            mat.CollisionResponse = collisionResponse;

            BlobAssetReference<Collider> asset;

            if (collider is UnityEngine.BoxCollider bc)
            {
                BoxGeometry geo = new()
                {
                    Size = new float3(bc.size) * new float3(bc.transform.localScale),
                    Orientation = quaternion.identity,
                    Center = float3.zero
                };

                asset = BoxCollider.Create(geo, CollisionFilter.Default, mat);
            }
            else if (collider is UnityEngine.SphereCollider sc)
            {
                float scale = math.max(math.max(collider.transform.localScale.x, collider.transform.localScale.y), collider.transform.localScale.z);

                SphereGeometry geo = new()
                {
                    Radius = sc.radius * scale,
                    Center = float3.zero
                };

                asset = SphereCollider.Create(geo, CollisionFilter.Default, mat);
            }
            else
            {
                throw new Exception($"Collider of type {collider.GetType()} not supported.");
            }

            store.TryAdd(ref asset);

            return asset;
        }
    }
}