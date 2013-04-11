#include "BulletPhysicsData.h"
#include "BulletDebugRenderer.h"
#include "XMLPhysicsData.h"
#include "utils.h"
#include <algorithm>

namespace GameEngine
{
	namespace PhysicsEngine
	{
		bool BulletPhysicsData::VInitializeSystems()
		{
			m_physicsMaterialData = new XMLPhysicsData();
			// TODO: does this need to be ../assets/materials.xml?
			m_physicsMaterialData->LoadDataFromXML("assets/materials.xml");

			SetupSystems();

			if (!m_pCollisionConfig || !m_pCollisionDispatcher ||
				!m_pCollisionBroadPhase || !m_pConstraintSolver ||
				!m_pDynamicsWorld)
			{
				return false;
			}

			// TODO: set debug drawer
			// m_pDynamicsWorld->setDebugDrawer(m_pDebugDrawer);

			m_pDynamicsWorld->setInternalTickCallback(BulletInternalTickCallback);
			m_pDynamicsWorld->setWorldUserInfo(this);

			return true;
		}

		BulletPhysicsData::~BulletPhysicsData()
		{
			CleanUpRigidBodies();
			CleanUpSystems();
			safe_delete(m_physicsMaterialData);
		}

		void BulletPhysicsData::SetupSystems()
		{
			m_pCollisionConfig = new btDefaultCollisionConfiguration();
			m_pCollisionDispatcher = new btCollisionDispatcher(m_pCollisionConfig);

			// Use an AABB tree for broad phase collision detection.
			m_pCollisionBroadPhase = new btDbvtBroadphase();

			// TODO: demo this
			m_pConstraintSolver = new btSequentialImpulseConstraintSolver();
			
			m_pDynamicsWorld = new btDiscreteDynamicsWorld(
				m_pCollisionDispatcher, m_pCollisionBroadPhase,
				m_pConstraintSolver, m_pCollisionConfig);

			m_pDebugRenderer = new BulletDebugRenderer();
		}

		void BulletPhysicsData::CleanUpSystems()
		{
			safe_delete(m_pDebugRenderer);
			safe_delete(m_pDynamicsWorld);
			safe_delete(m_pConstraintSolver);
			safe_delete(m_pCollisionBroadPhase);
			safe_delete(m_pCollisionDispatcher);
			safe_delete(m_pCollisionConfig);
		}

		void BulletPhysicsData::CleanUpRigidBodies()
		{
			// Iterate backwards to avoid linear-time deletes.
			auto collisionObjects = m_pDynamicsWorld->getCollisionObjectArray();
			int i = m_pDynamicsWorld->getNumCollisionObjects();
			while (i >= 0)
			{
				RemoveCollisionObject(collisionObjects[i]);
				--i;
			}
			m_actorToRigidBodyMap.clear();
			m_rigidBodyToActorMap.clear();
		}

		void BulletPhysicsData::RemoveCollisionObject(btCollisionObject *obj)
		{
			for (auto it = m_PreviousTickCollisions.begin(); it != m_PreviousTickCollisions.end(); )
			{
				if ((*it).first == obj || (*it).second == obj)
				{
					m_PreviousTickCollisions.erase(it++);
				}
				else
				{
					++it;
				}
			}

			btRigidBody* body = btRigidBody::upcast(obj);
			if (body)
			{
				delete body->getMotionState();
				delete body->getCollisionShape();
				delete body->getUserPointer(); // deletes the scene node that uses this rigid body

				// destroy related constraints
				for (int i = body->getNumConstraintRefs()-1; i >= 0; i++)
				{
					auto constraintRef = body->getConstraintRef(i);
					m_pDynamicsWorld->removeConstraint(constraintRef);
					delete constraintRef;
				}
			}

			m_pDynamicsWorld->removeCollisionObject(obj);

			delete obj;
		}

		void BulletPhysicsData::BulletInternalTickCallback(
			btDynamicsWorld * const pWorld, const btScalar timeStep)
		{
			// TODO
		}
	}
}