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
			virtual bool VUpdateSimulation(float deltaSec) = 0;
			virtual bool VSyncScene() = 0;

			// Initializing different physics world objects.
			// TODO: extend with additional types. E.g.
			// boxes, triangle meshes etc.
			// TODO: add materials etc.
			// (As an enumeration? Reading from an XML file?)
			virtual void VAddSphere(float radius, WeakActorPtr actor,
				const LinearAlgebra::Mat4& initialTransform) = 0;

			virtual void VRemoveActor(ActorID id) = 0;

			virtual void VRenderDiagnostics() = 0;

			// TODO: physics world modifiers here (see book, page 589)
			virtual void VCreateTrigger(WeakActorPtr gameActor, const LinearAlgebra::Vec3& pos, const float dim) = 0;
		};
	}
}

#endif