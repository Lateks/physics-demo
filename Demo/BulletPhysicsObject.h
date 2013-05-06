#pragma once

#include "enginefwd.h"
#include "BulletPhysicsConstraint.h"
#include <btBulletCollisionCommon.h>
#include <btBulletDynamicsCommon.h>
#include <vector>
#include <map>

namespace GameEngine
{
	namespace Physics
	{
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

			ConstraintID AddConstraint(btTypedConstraint *pConstraint,
				BulletPhysicsConstraint::ConstraintType constraintType =
				BulletPhysicsConstraint::ConstraintType::BASIC_CONSTRAINT);
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