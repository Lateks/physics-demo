#ifndef I_PHYSICS_ENGINE
#define I_PHYSICS_ENGINE

#include "enginefwd.h"
#include <string>

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
			// TODO: extend with additional types. E.g.
			// boxes, triangle meshes etc.
			virtual void VAddSphere(float radius, WeakActorPtr actor,
				const std::string& density, const std::string& material) = 0;
			virtual void VAddBox(const LinearAlgebra::Vec3& dimensions, WeakActorPtr actor,
				const std::string& density, const std::string& material) = 0;

			virtual void VRemoveActor(ActorID id) = 0;

			virtual void VRenderDiagnostics() = 0;

			// TODO: physics world modifiers here (see book, page 589)
			virtual void VCreateTrigger(WeakActorPtr gameActor, const LinearAlgebra::Vec3& pos, const float dim) = 0;
		};
	}
}

#endif