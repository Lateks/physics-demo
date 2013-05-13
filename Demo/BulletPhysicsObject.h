#pragma once

#include "enginefwd.h"
#include "IPhysicsEngine.h"
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
			typedef IPhysicsEngine::PhysicsObjectType ObjectType;

			BulletPhysicsObject(ActorID actorId, ObjectType type = ObjectType::DYNAMIC)
				: m_type(type), m_actorId(actorId) { }

			ActorID GetActorId()
			{
				return m_actorId;
			}

			void SetPhysicsType(ObjectType type)
			{
				m_type = type;
			}

			bool IsDynamic() const
			{
				return m_type == ObjectType::DYNAMIC;
			}

			bool IsKinematic() const
			{
				return m_type == ObjectType::KINEMATIC;
			}

			bool IsTrigger() const
			{
				return m_type == ObjectType::TRIGGER;
			}

			bool IsStatic() const
			{
				return m_type == ObjectType::TRIGGER || m_type == ObjectType::STATIC;
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
		private:
			ObjectType m_type;
			ActorID m_actorId;
			std::vector<btRigidBody*> m_rigidBodies;
		};
	}
}