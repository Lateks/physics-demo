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
			BulletPhysics();
			virtual ~BulletPhysics();
			
			virtual bool VInitEngine() override;
			virtual void VSyncScene() override;
			virtual void VUpdateSimulation(float deltaSec) override;

			virtual void VAddSphere(float radius, WeakActorPtr pActor,
				const std::string& density, const std::string& material) override;
			virtual void VAddBox(const Vec3& dimensions, WeakActorPtr pActor,
				const std::string& density, const std::string& material) override;
			virtual void VAddConvexMesh(std::vector<Vec3>& vertices,
				WeakActorPtr pActor, const std::string& density, const std::string& material) override;
			virtual void VAddConvexStaticColliderMesh(std::vector<Vec3>& vertices, WeakActorPtr pActor) override;
			virtual void VAddConvexStaticColliderMesh(std::vector<Vec4>& planeEquations, WeakActorPtr pActor) override;
			virtual void VAddBspMap(BspLoader& bspLoad, WeakActorPtr pActor) override;

			virtual void VRemoveActor(ActorID id) override;

			virtual void VRenderDiagnostics() override;

			virtual void VCreateTrigger(WeakActorPtr pActor, const float dim) override;
			virtual void VApplyForce(const Vec3& direction, float newtons, ActorID id) override;
			virtual void VApplyTorque(const Vec3& direction, float magnitude, ActorID id) override;
			virtual void VStopActor(ActorID id) override;
			virtual void VSetVelocity(ActorID id, const Vec3& newVelocity) override;
			virtual void VSetGlobalGravity(Vec3& gravity) override;
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