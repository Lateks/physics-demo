#include "BulletPhysicsData.h"
#include "BulletDebugRenderer.h"
#include "GameActor.h"
#include "WorldTransformComponent.h"
#include "XMLPhysicsData.h"
#include "BulletConversions.h"
#include "utils.h"
#include <algorithm>
#include <memory>
#include <cassert>

namespace GameEngine
{
	using LinearAlgebra::Mat4;

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

		void BulletPhysicsData::AddShape(StrongActorPtr pActor, btCollisionShape *shape,
			float mass, const std::string& material)
		{
			assert(pActor.get());
			// There can be only one rigid body per actor in this implementation.
			ActorID id = pActor->GetID();
			assert(m_actorToRigidBodyMap.find(id) == m_actorToRigidBodyMap.end());

			MaterialData matData(m_physicsMaterialData->LookupMaterial(material));

			btVector3 localInertia;
			if (mass > 0.f)
				shape->calculateLocalInertia(mass, localInertia);

			std::weak_ptr<WorldTransformComponent> pWeakWorldTransform
				= pActor->GetWorldTransform();
			if (!pWeakWorldTransform.expired())
			{
				std::shared_ptr<WorldTransformComponent> pWorldTransform(pWeakWorldTransform);
				btQuaternion rotation(Quaternion_to_btQuaternion(pWorldTransform->GetRotation()));
				btVector3 translation(Vec3_to_btVector3(pWorldTransform->GetPosition()));
				btTransform transform(rotation, translation);
				btMotionState *motionState = new btDefaultMotionState(transform);

				btRigidBody::btRigidBodyConstructionInfo rbInfo(
					mass, motionState, shape, localInertia);

				rbInfo.m_restitution = matData.m_restitution;
				rbInfo.m_friction = matData.m_friction;

				btRigidBody * const body = new btRigidBody(rbInfo);
				m_pDynamicsWorld->addRigidBody(body);

				m_actorToRigidBodyMap[id] = body;
				m_rigidBodyToActorMap[body] = id;
			}
		}
	}
}