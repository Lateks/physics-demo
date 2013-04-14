#ifndef BULLET_PHYSICS
#define BULLET_PHYSICS

#include "enginefwd.h"
#include "IPhysicsEngine.h"

namespace GameEngine
{
	namespace PhysicsEngine
	{
		class BulletPhysics : public IPhysicsEngine
		{
		public:
			BulletPhysics() { }
			virtual ~BulletPhysics();
			
			virtual bool VInitEngine() override;
			virtual void VSyncScene() override;
			virtual void VUpdateSimulation(float deltaSec) override;

			virtual void VAddSphere(float radius, WeakActorPtr pActor,
				const std::string& density, const std::string& material) override;
			virtual void VAddBox(const LinearAlgebra::Vec3& dimensions, WeakActorPtr pActor,
				const std::string& density, const std::string& material) override;
			virtual void VAddConvexMesh(std::vector<LinearAlgebra::Vec3>& vertices,
				WeakActorPtr pActor, const std::string& density, const std::string& material) override;

			virtual void VRemoveActor(ActorID id) override;

			virtual void VRenderDiagnostics() override;

			virtual void VCreateTrigger(WeakActorPtr pActor, const LinearAlgebra::Vec3& pos,
				const float dim) override;
		private:
			// The VS11 C++ compiler does not yet support deleting
			// constructors, so make these private to make the class
			// "non-copyable".
			BulletPhysics(const BulletPhysics& other);
			BulletPhysics& operator=(const BulletPhysics& other);
			BulletPhysicsData *m_pData;
		};
	}
}

#endif