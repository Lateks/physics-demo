#ifndef BULLET_PHYSICS_OBJECT_H
#define BULLET_PHYSICS_OBJECT_H

#include "enginefwd.h"
#include "IEventManager.h"
#include <btBulletCollisionCommon.h>
#include <btBulletDynamicsCommon.h>
#include <vector>
#include <map>

namespace GameEngine
{
	namespace Physics
	{
		class BulletPhysicsConstraint
		{
		public:
			BulletPhysicsConstraint(btTypedConstraint *constraint)
				: m_pConstraint(constraint), m_updaterEventType(Events::EventType::NONE) { }
			BulletPhysicsConstraint(btTypedConstraint *constraint, Events::EventHandlerPtr handler, Events::EventType type)
				: m_pConstraint(constraint), m_pConstraintUpdater(handler), m_updaterEventType(type) { }
			~BulletPhysicsConstraint();
			BulletPhysicsConstraint(BulletPhysicsConstraint&& other);
			BulletPhysicsConstraint& operator=(BulletPhysicsConstraint&& other);

			Events::EventHandlerPtr GetConstraintUpdater()
			{
				return m_pConstraintUpdater;
			}
			Events::EventType GetHandlerEventType()
			{
				return m_updaterEventType;
			}
			btTypedConstraint *GetBulletConstraint()
			{
				return m_pConstraint;
			}
			void SetConstraintUpdater(Events::EventHandlerPtr updater, Events::EventType eventType)
			{
				m_pConstraintUpdater = updater;
				m_updaterEventType = eventType;
			}
		private:
			BulletPhysicsConstraint(BulletPhysicsConstraint& other);
			BulletPhysicsConstraint& operator=(BulletPhysicsConstraint& other);
			btTypedConstraint *m_pConstraint;
			Events::EventHandlerPtr m_pConstraintUpdater;
			Events::EventType m_updaterEventType;
		};

		class BulletPhysicsObject
		{
		public:
			enum class PhysicsType
			{
				STATIC,
				DYNAMIC,
				KINEMATIC,
				TRIGGER // is also static
			};

			BulletPhysicsObject(ActorID actorId, PhysicsType type = PhysicsType::DYNAMIC)
				: m_type(type), m_actorId(actorId) { }

			ActorID GetActorId()
			{
				return m_actorId;
			}

			void SetPhysicsType(PhysicsType type)
			{
				m_type = type;
			}

			bool IsDynamic() const
			{
				return m_type == PhysicsType::DYNAMIC;
			}

			bool IsKinematic() const
			{
				return m_type == PhysicsType::KINEMATIC;
			}

			bool IsTrigger() const
			{
				return m_type == PhysicsType::TRIGGER;
			}

			bool IsStatic() const
			{
				return m_type == PhysicsType::TRIGGER || m_type == PhysicsType::STATIC;
			}

			const std::vector<btRigidBody*>& GetRigidBodies() const
			{
				return m_rigidBodies;
			}

			void AddRigidBody(btRigidBody *pBody)
			{
				m_rigidBodies.push_back(pBody);
			}

			size_t GetNumBodies()
			{
				return m_rigidBodies.size();
			}

			ConstraintID AddConstraint(btTypedConstraint *pConstraint);
			void RemoveConstraint(ConstraintID id);

			std::shared_ptr<BulletPhysicsConstraint> GetConstraint(ConstraintID id);
			unsigned int GetNumConstraints()
			{
				return m_pConstraints.size();
			}
		private:
			static ConstraintID constraintId;
			PhysicsType m_type;
			ActorID m_actorId;
			std::vector<btRigidBody*> m_rigidBodies;
			std::map<ConstraintID, std::shared_ptr<BulletPhysicsConstraint>> m_pConstraints;
		};
	}
}

#endif