#ifndef BULLET_PHYSICS
#define BULLET_PHYSICS

#include "enginefwd.h"
#include "IPhysicsEngine.h"

namespace GameEngine
{
	namespace PhysicsEngine
	{
		// TODO: make noncopyable (boost?)
		class BulletPhysics : public IPhysicsEngine
		{
		public:
			BulletPhysics() { }
			virtual ~BulletPhysics();
			
			virtual bool VInitEngine() override;
			virtual void VSyncScene() override;
			virtual void VUpdateSimulation(float deltaSec) override;

			virtual void VAddSphere(float radius, WeakActorPtr pActor,
				const LinearAlgebra::Mat4& initialTransform) override;

			virtual void VRemoveActor(ActorID id) override;

			virtual void VRenderDiagnostics() override;

			virtual void VCreateTrigger(WeakActorPtr pActor, const LinearAlgebra::Vec3& pos,
				const float dim) override;
		private:
			BulletPhysicsData *m_pData;
		};
	}
}

#endif