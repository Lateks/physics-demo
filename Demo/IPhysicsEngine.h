#pragma once

#include "enginefwd.h"
#include "BSPLoader.h"
#include <string>
#include <vector>

/* This header file defines an interface for Physics engines
 * similar to the one presented by McShaffry in Game Coding
 * Complete 4th ed. starting on page 589.
 */
namespace GameEngine
{
	namespace Physics
	{
		class IPhysicsEngine
		{
		public:
			enum class PhysicsObjectType
			{
				DYNAMIC,
				STATIC,
				TRIGGER,
				KINEMATIC
			};

			virtual ~IPhysicsEngine() { }

			// Initialization routines.
			virtual bool VInitEngine(const std::string& materialFileName) = 0;

			// Updating the simulation.
			virtual void VUpdateSimulation(float deltaSec) = 0;
			virtual void VSyncScene() = 0;

			/* Initializing different physics world objects.
			 * Density and material strings are used only if the PhysicsObjectType is
			 * dynamic. (Would possibly be used for kinematic objects as well but these
			 * are not actually implemented in the BulletPhysics class at the moment.)
			 */
			virtual void VAddSphere(ActorPtr pActor, float radius, PhysicsObjectType type,
				const std::string& density = "", const std::string& material = "") = 0;
			virtual void VAddBox(ActorPtr pActor, const Vec3& dimensions, PhysicsObjectType type,
				const std::string& density = "", const std::string& material = "") = 0;
			virtual void VAddConvexMesh(ActorPtr pActor, std::vector<Vec3>& vertices,
				PhysicsObjectType type, const std::string& density = "", const std::string& material = "") = 0;
			virtual void VAddConvexMesh(ActorPtr pActor, std::vector<Vec4>& planeEquations,
				PhysicsObjectType type, const std::string& density = "", const std::string& material = "") = 0;

			// Adding e.g. Quake maps from bsp files.
			virtual void VLoadBspMap(BspLoader& bspLoad, ActorPtr pActor, const std::string& material) = 0;

			virtual void VRemoveActor(ActorID id) = 0;

			virtual void VApplyForce(const Vec3& direction, float magnitude, ActorID id) = 0;
			virtual void VApplyTorque(const Vec3& direction, float magnitude, ActorID id) = 0;
			virtual void VStopActor(ActorID id) = 0; // set velocity to 0
			virtual void VSetLinearVelocity(ActorID id, const Vec3& direction, float magnitude) = 0;
			virtual Vec3 VGetLinearVelocity(ActorID id) = 0;
			virtual void VSetAngularVelocity(ActorID id, const Vec3& rotationAxis, float radiansPerSecond) = 0;
			virtual Vec3 VGetAngularVelocity(ActorID id) = 0;
			virtual void VSetAngularFactor(ActorID id, const Vec3& factor) = 0; // can be used to e.g. disable rotations
			virtual Vec3 VGetAngularFactor(ActorID id) = 0;

			virtual void VSetGlobalGravity(Vec3& gravity) = 0;
			virtual Vec3 VGetGlobalGravity() = 0;

			/* Uses a ray cast to determine the closest actor intersected by
			 * the ray. Returns 0 if no actor was hit. The last parameter is an
			 * output vector specifying the "picking point" of the actor. This is
			 * the point where the ray intersects the rigid body associated with
			 * the actor. Only dynamic bodies are picked by this method.
			 */
			virtual ActorID VGetClosestActorHit(Vec3& rayFrom, Vec3& rayTo, Vec3& pickPosition) const = 0;

			// Adds a DOF6 constraint, constraining the degrees of freedom of the object.
			// This is useful for e.g. constraining an object to camera or mouse movement.
			virtual ConstraintID VAddDOF6Constraint(ActorID actorID, const Vec3& pivotPosition) = 0;
			virtual void VUpdateDOF6PivotPoint(ConstraintID constraintId, const Vec3& pivotPosition) = 0;
			virtual void VRemoveConstraint(ConstraintID constraintId) = 0;
			//virtual void VRemoveConstraints(ActorID actorID) = 0;
		};

		class IPhysicsEngineFactory
		{
		public:
			virtual ~IPhysicsEngineFactory() { }
			virtual std::shared_ptr<IPhysicsEngine> CreatePhysicsEngine() = 0;
		};
	}
}