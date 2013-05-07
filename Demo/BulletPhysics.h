#pragma once

#include "enginefwd.h"
#include "IPhysicsEngine.h"

namespace GameEngine
{
	namespace Physics
	{
		struct BulletPhysicsData;

		class BulletPhysics : public IPhysicsEngine
		{
		public:
			explicit BulletPhysics(float worldScale = 1.f);
			virtual ~BulletPhysics();
			
			virtual bool VInitEngine() override;
			virtual void VSyncScene() override;
			virtual void VUpdateSimulation(float deltaSec) override;

			virtual void VAddSphere(float radius, ActorPtr pActor,
				const std::string& density, const std::string& material) override;
			virtual void VAddBox(const Vec3& dimensions, ActorPtr pActor,
				const std::string& density, const std::string& material) override;
			virtual void VAddConvexMesh(std::vector<Vec3>& vertices,
				ActorPtr pActor, const std::string& density, const std::string& material) override;
			virtual void VAddConvexStaticColliderMesh(std::vector<Vec3>& vertices, ActorPtr pActor) override;
			virtual void VAddConvexStaticColliderMesh(std::vector<Vec4>& planeEquations, ActorPtr pActor) override;
			virtual void VCreateTrigger(ActorPtr pActor, const float dim) override;

			virtual void VLoadBspMap(BspLoader& bspLoad, ActorPtr pActor) override;

			virtual void VRemoveActor(ActorID id) override;

			virtual void VApplyForce(const Vec3& direction, float newtons, ActorID id) override;
			virtual void VApplyTorque(const Vec3& direction, float magnitude, ActorID id) override;
			virtual void VStopActor(ActorID id) override;
			virtual void VSetLinearVelocity(ActorID id, const Vec3& direction, float magnitude) override;
			virtual void VSetAngularVelocity(ActorID id, const Vec3& rotationAxis, float magnitude) override;

			virtual void VSetGlobalGravity(Vec3& gravity) override;

			virtual ActorID VGetClosestActorHit(Vec3& rayFrom, Vec3& rayTo, Vec3& pickPosition) const override;
			virtual ConstraintID VAddPickConstraint(ActorID actorID, Vec3& pickPosition, Vec3& cameraPosition) override;
			virtual void VUpdatePickConstraint(ActorID actorId, ConstraintID constraintId, Vec3& rayFrom, Vec3& rayTo) override;
			virtual void VRemoveConstraint(ActorID actorID, unsigned int constraintId) override;
		private:
			// The VS11 C++ compiler does not yet support deleting
			// constructors, so make these private to make the class
			// "non-copyable".
			BulletPhysics(const BulletPhysics& other);
			BulletPhysics& operator=(const BulletPhysics& other);

			std::unique_ptr<BulletPhysicsData> m_pData;
		};
	}
}