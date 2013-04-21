#ifndef I_PHYSICS_ENGINE
#define I_PHYSICS_ENGINE

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
			virtual ~IPhysicsEngine() { }

			// Initialization routines.
			virtual bool VInitEngine() = 0;

			// Updating the simulation.
			virtual void VUpdateSimulation(float deltaSec) = 0;
			virtual void VSyncScene() = 0;

			// Initializing different physics world objects.
			virtual void VAddSphere(float radius, WeakActorPtr pActor,
				const std::string& density, const std::string& material) = 0;
			virtual void VAddBox(const Vec3& dimensions, WeakActorPtr pActor,
				const std::string& density, const std::string& material) = 0;
			virtual void VAddConvexMesh(std::vector<Vec3>& vertices,
				WeakActorPtr pActor, const std::string& density, const std::string& material) = 0;

			// These are used to add e.g. map parts and other static entities.
			virtual void VAddConvexStaticColliderMesh(std::vector<Vec3>& vertices, WeakActorPtr pActor) = 0;
			virtual void VAddConvexStaticColliderMesh(std::vector<Vec4>& planeEquations, WeakActorPtr pActor) = 0;

			virtual void VCreateTrigger(WeakActorPtr gameActor, const float dim) = 0;

			// Adding e.g. Quake maps from bsp files.
			virtual void VLoadBspMap(BspLoader& bspLoad, WeakActorPtr pActor) = 0;

			virtual void VRemoveActor(ActorID id) = 0;

			virtual void VApplyForce(const Vec3& direction, float newtons, ActorID id) = 0;
			virtual void VApplyTorque(const Vec3& direction, float magnitude, ActorID id) = 0;
			virtual void VStopActor(ActorID id) = 0; // set velocity to 0
			virtual void VSetLinearVelocity(ActorID id, const Vec3& direction, float magnitude) = 0;
			virtual void VSetAngularVelocity(ActorID id, const Vec3& rotationAxis, float magnitude) = 0;

			virtual void VSetGlobalGravity(Vec3& gravity) = 0;

			// Uses a ray cast to determine the closest actor intersected by
			// the ray. Returns 0 if no actor was hit.
			virtual ActorID GetClosestActorHit(Vec3& rayFrom, Vec3& rayTo) const = 0;
		};
	}
}

#endif