#include "BulletPhysicsData.h"
#include "BulletPhysics.h"
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
	namespace PhysicsEngine
	{
		btRigidBody *BulletPhysicsData::GetRigidBody(ActorID id) const
		{
			auto it = m_actorToRigidBodyMap.find(id);
			if (it != m_actorToRigidBodyMap.end())
				return it->second;
			return nullptr;
		}

		bool BulletPhysicsData::VInitializeSystems()
		{
			m_physicsMaterialData = new XMLPhysicsData();
			m_physicsMaterialData->LoadDataFromXML("..\assets\materials.xml");

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

		// Removes the collision pairs that contain the given collision object.
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
			assert(pWorld);
			assert(pWorld->getWorldUserInfo());
			BulletPhysics * const pBulletPhysics =
				static_cast<BulletPhysics*>(pWorld->getWorldUserInfo());
			BulletPhysicsData * const pData = pBulletPhysics->m_pData;

			btDispatcher *const pDispatcher = pWorld->getDispatcher();
			CollisionPairs currentTickCollisions;
			HandleNewCollisions(pData, pDispatcher, currentTickCollisions);

			// Get collisions that existed on the last tick but not on this tick.
			CollisionPairs removedCollisions;
			std::set_difference(pData->m_PreviousTickCollisions.begin(),
								pData->m_PreviousTickCollisions.end(),
								currentTickCollisions.begin(), currentTickCollisions.end(),
								std::inserter(removedCollisions, removedCollisions.begin()));

			std::for_each(removedCollisions.begin(), removedCollisions.end(),
				[&pData] (const CollisionPair& pair)
			{
				const btRigidBody * const body1 = pair.first;
				const btRigidBody * const body2 = pair.second;

				pData->SendSeparationEvent(body1, body2);
			});

			pData->m_PreviousTickCollisions = currentTickCollisions;
		}

		void BulletPhysicsData::HandleNewCollisions(BulletPhysicsData *pData,
			btDispatcher *pDispatcher, CollisionPairs& currentTickCollisions)
		{
			for (int manifoldIdx = 0; manifoldIdx<pDispatcher->getNumManifolds(); ++manifoldIdx)
			{
				const btPersistentManifold * const pContactPoint =
					pDispatcher->getManifoldByIndexInternal(manifoldIdx);
				assert(pContactPoint);
				if (!pContactPoint)
					continue;

				const btRigidBody * body1 =
					static_cast<btRigidBody const *>(pContactPoint->getBody0());
				const btRigidBody * body2 =
					static_cast<btRigidBody const *>(pContactPoint->getBody1());

				if (body1 > body2)
				{
					// TODO: does this work right? (swaps the pointers, not their contents)
					std::swap(body1, body2);
				}

				const CollisionPair newPair = std::make_pair(body1, body2);
				currentTickCollisions.insert(newPair);

				if (pData->m_PreviousTickCollisions.find(newPair)
					== pData->m_PreviousTickCollisions.end())
				{
					pData->SendNewCollisionEvent(pContactPoint, body1, body2);
				}
			}
		}

		void BulletPhysicsData::AddShape(StrongActorPtr pActor, btCollisionShape *shape,
			float mass, const std::string& material)
		{
			assert(pActor.get());
			// There can be only one rigid body per actor in this implementation.
			ActorID id = pActor->GetID();
			assert(m_actorToRigidBodyMap.find(id) == m_actorToRigidBodyMap.end());

			std::weak_ptr<WorldTransformComponent> pWeakWorldTransform
				= pActor->GetWorldTransform();
			if (!pWeakWorldTransform.expired())
			{
				std::shared_ptr<WorldTransformComponent> pWorldTransform(pWeakWorldTransform);
				btMotionState *motionState = GetMotionStateFrom(pWorldTransform);

				MaterialData matData(m_physicsMaterialData->LookupMaterial(material));

				btVector3 localInertia;
				if (mass > 0.f)
					shape->calculateLocalInertia(mass, localInertia);

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

		void BulletPhysicsData::AddTriggerShape(StrongActorPtr pActor, btCollisionShape *shape)
		{
			assert(pActor.get());
			ActorID id = pActor->GetID();
			assert(m_actorToRigidBodyMap.find(id) == m_actorToRigidBodyMap.end());

			std::weak_ptr<WorldTransformComponent> pWeakWorldTransform
				= pActor->GetWorldTransform();
			if (!pWeakWorldTransform.expired())
			{
				std::shared_ptr<WorldTransformComponent> pWorldTransform(pWeakWorldTransform);
				btMotionState *motionState = GetMotionStateFrom(pWorldTransform);

				// Objects that have 0 mass are regarded as immovable by Bullet.
				btScalar const mass = 0;
				btRigidBody::btRigidBodyConstructionInfo rbInfo(
					mass, motionState, shape, btVector3(0, 0, 0));
				btRigidBody * const body = new btRigidBody(rbInfo);

				m_pDynamicsWorld->addRigidBody(body);

				// Triggers should not be taken into account in collision detection.
				body->setCollisionFlags(
					body->getCollisionFlags() | btRigidBody::CF_NO_CONTACT_RESPONSE);

				m_actorToRigidBodyMap[id] = body;
				m_rigidBodyToActorMap[body] = id;
			}
		}

		btMotionState *BulletPhysicsData::GetMotionStateFrom(std::shared_ptr<WorldTransformComponent> pWorldTransform)
		{
			btQuaternion rotation(Quaternion_to_btQuaternion(pWorldTransform->GetRotation()));
			btVector3 translation(Vec3_to_btVector3(pWorldTransform->GetPosition()));
			btTransform transform(rotation, translation);
			return new btDefaultMotionState(transform);
		}

		void BulletPhysicsData::SendNewCollisionEvent(const btPersistentManifold * manifold,
			const btRigidBody * pBody1, const btRigidBody * pBody2)
		{
			// TODO
		}

		void BulletPhysicsData::SendSeparationEvent(const btRigidBody * pBody1, const btRigidBody * pBody2)
		{
			// TODO
		}
	}
}