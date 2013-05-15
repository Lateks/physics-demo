#pragma once

#include "enginefwd.h"
#include "IPhysicsEngine.h"

namespace GameEngine
{
	namespace Physics
	{
		class BulletPhysicsFactory : public IPhysicsEngineFactory
		{
		public:
			/* Constructor parameters:
			   - materialFilePath is the path to the XML file containing material data.
			   - worldScale is the scaling factor used for the world. This is used to scale
			     all parameters (object sizes, forces etc.) that come into the system, except
			     those that do not need to be scaled (e.g. radians per second are the same
			     regardless of the world scale). Bullet documentation recommends scaling
			     the world so that 1 unit (1.0f) equals about a single metre.
			   - contactThreshold affects the sending of collision and separation events:
			     a collision detected by Bullet only results in an event if the contact point
			     distance is smaller than this threshold (and vice versa for the separation event).
			     This is used because Bullet actually detects the collision before there is an actual
			     contact. This is scaled by the worldScale parameter.
			   - collisionMargin affects the collision system margins on convex collision objects
			     (but apparently not e.g. spheres or boxes - in these the margin is embedded in the
			     object). This is also scaled by the worldScale factor. Bullet documentation advices
			     against setting this to zero to keep the collision system stable.
			 */
			BulletPhysicsFactory(const std::string& materialFilePath,
				float worldScale = 1.f, float contactThreshold = 0.1f, float collisionMargin = 0.3f);
			virtual ~BulletPhysicsFactory() { }
			virtual std::shared_ptr<IPhysicsEngine> CreatePhysicsEngine() const override;
		private:
			const std::string m_materialFilePath;
			float m_worldScale;
			float m_contactThreshold;
			float m_collisionMargin;
		};

		struct BulletPhysicsData;

		class BulletPhysics : public IPhysicsEngine
		{
		public:
			friend class BulletPhysicsFactory;

			virtual ~BulletPhysics();
			
			virtual bool VInitEngine(const std::string& materialFileName) override;
			virtual void VSyncScene() override;
			virtual void VUpdateSimulation(float deltaSec) override;

			virtual void VAddSphere(ActorPtr pActor, float radius, PhysicsObjectType type,
				const std::string& density = "", const std::string& material = "") override;
			virtual void VAddBox(ActorPtr pActor, const Vec3& dimensions, PhysicsObjectType type,
				const std::string& density = "", const std::string& material = "") override;
			virtual void VAddConvexMesh(ActorPtr pActor, std::vector<Vec3>& vertices,
				PhysicsObjectType type, const std::string& density = "", const std::string& material = "") override;
			virtual void VAddConvexMesh(ActorPtr pActor, std::vector<Vec4>& planeEquations,
				PhysicsObjectType type, const std::string& density = "", const std::string& material = "") override;

			virtual void VLoadBspMap(ActorPtr pActor, BspLoader& bspLoad, const std::string& material) override;

			virtual void VRemoveActor(ActorID id) override;

			virtual void VApplyForce(ActorID id, const Vec3& direction, float magnitude) override;
			virtual void VApplyTorque(ActorID id, const Vec3& direction, float magnitude) override;
			virtual void VStopActor(ActorID id) override;
			virtual void VSetLinearVelocity(ActorID id, const Vec3& direction, float magnitude) override;
			virtual Vec3 VGetLinearVelocity(ActorID id) override;
			virtual void VSetAngularVelocity(ActorID id, const Vec3& rotationAxis, float radiansPerSecond) override;
			virtual Vec3 VGetAngularVelocity(ActorID id) override;
			virtual void VSetAngularFactor(ActorID id, const Vec3& factor);
			virtual Vec3 VGetAngularFactor(ActorID id) override;

			virtual void VSetGlobalGravity(Vec3& gravity) override;
			virtual Vec3 VGetGlobalGravity() override;

			virtual ActorID VGetClosestActorHit(Vec3& rayFrom, Vec3& rayTo, Vec3& hitPosition) const override;

			virtual ConstraintID VAddDOF6Constraint(ActorID actorID, const Vec3& pivotPosition) override;
			virtual void VUpdateDOF6PivotPoint(ConstraintID constraintId, const Vec3& pivotPosition) override;
			virtual void VRemoveConstraint(ConstraintID constraintId) override;
		private:
			// This can only be created through the factory. This prevents using the system
			// without initializing it first. (The factory performs initialization.)
			BulletPhysics(float worldScale, float contactThreshold, float collisionMargin);

			// The VS11 C++ compiler does not yet support deleting
			// constructors, so make these private to make the class
			// "non-copyable".
			BulletPhysics(const BulletPhysics& other);
			BulletPhysics& operator=(const BulletPhysics& other);

			std::shared_ptr<BulletPhysicsData> m_pData;
		};
	}
}