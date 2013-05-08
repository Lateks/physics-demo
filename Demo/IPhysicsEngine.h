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
			virtual bool VInitEngine() = 0;

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

			virtual void VApplyForce(const Vec3& direction, float newtons, ActorID id) = 0;
			virtual void VApplyTorque(const Vec3& direction, float magnitude, ActorID id) = 0;
			virtual void VStopActor(ActorID id) = 0; // set velocity to 0
			virtual void VSetLinearVelocity(ActorID id, const Vec3& direction, float magnitude) = 0;
			virtual void VSetAngularVelocity(ActorID id, const Vec3& rotationAxis, float magnitude) = 0;

			virtual void VSetGlobalGravity(Vec3& gravity) = 0;

			/* Uses a ray cast to determine the closest actor intersected by
			 * the ray. Returns 0 if no actor was hit. The last parameter is an
			 * output vector specifying the "picking point" of the actor. This is
			 * the point where the ray intersects the rigid body associated with
			 * the actor. Only dynamic bodies are picked by this method.
			 */
			virtual ActorID VGetClosestActorHit(Vec3& rayFrom, Vec3& rayTo, Vec3& pickPosition) const = 0;
			virtual unsigned int VAddPickConstraint(ActorID actorID, Vec3& pickPosition, Vec3& cameraPosition) = 0;
			virtual void VUpdatePickConstraint(ActorID actorId, ConstraintID constraintId, Vec3& rayFrom, Vec3& rayTo) = 0;
			virtual void VRemoveConstraint(ActorID actorID, unsigned int constraintId) = 0;
		};
	}
}