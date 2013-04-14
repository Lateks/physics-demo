#ifndef I_PHYSICS_ENGINE
#define I_PHYSICS_ENGINE

#include "enginefwd.h"
#include <string>
#include <vector>

/* This header file defines an interface for Physics engines
 * similar to the one presented by McShaffry in Game Coding
 * Complete 4th ed. starting on page 589.
 */
namespace GameEngine
{
	namespace PhysicsEngine
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
			virtual void VAddBox(const LinearAlgebra::Vec3& dimensions, WeakActorPtr pActor,
				const std::string& density, const std::string& material) = 0;
			virtual void VAddConvexMesh(std::vector<LinearAlgebra::Vec3>& vertices,
				WeakActorPtr pActor, const std::string& density, const std::string& material) = 0;

			virtual void VRemoveActor(ActorID id) = 0;

			virtual void VRenderDiagnostics() = 0;

			virtual void VCreateTrigger(WeakActorPtr gameActor, const float dim) = 0;
			virtual void VApplyForce(const LinearAlgebra::Vec3& direction, float newtons, ActorID id) = 0;
			virtual void VApplyTorque(const LinearAlgebra::Vec3& direction, float magnitude, ActorID id) = 0;
			virtual void VStopActor(ActorID id) = 0;
			virtual void VSetVelocity(ActorID id, const LinearAlgebra::Vec3& newVelocity) = 0;
		};
	}
}

#endif