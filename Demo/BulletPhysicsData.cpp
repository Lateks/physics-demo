#include "BulletPhysicsData.h"
#include "BulletPhysics.h"
#include "BulletDebugRenderer.h"
#include "GameActor.h"
#include "GameData.h"
#include "IEventManager.h"
#include "Events.h"
#include "WorldTransformComponent.h"
#include "XMLPhysicsData.h"
#include "BulletConversions.h"
#include "utils.h"
#include <algorithm>
#include <memory>
#include <cassert>
#include <iostream> // TODO: remove when no longer needed for quick debugging

namespace GameEngine
{
	namespace PhysicsEngine
	{
		std::vector<btRigidBody*> BulletPhysicsData::GetRigidBodies(ActorID id) const
		{
			auto it = m_actorToRigidBodyListMap.find(id);
			if (it != m_actorToRigidBodyListMap.end())
				return it->second;
			return std::vector<btRigidBody*>();
		}

		ActorID BulletPhysicsData::GetActorID(const btRigidBody *pBody) const
		{
			auto it = m_rigidBodyToActorMap.find(pBody);
			assert(it != m_rigidBodyToActorMap.end());
			if (it != m_rigidBodyToActorMap.end())
				return it->second;
			return 0;
		}

		bool BulletPhysicsData::VInitializeSystems()
		{
			m_physicsMaterialData = new XMLPhysicsData();
			m_physicsMaterialData->LoadDataFromXML("..\\assets\\materials.xml");

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

			//m_pDebugRenderer = new BulletDebugRenderer();
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
			int idx = m_pDynamicsWorld->getNumCollisionObjects() - 1;
			while (idx >= 0)
			{
				RemoveCollisionObject(collisionObjects[idx]);
				--idx;
			}
			m_actorToRigidBodyListMap.clear();
			m_rigidBodyToActorMap.clear();
		}

		// Removes the collision pairs that contain the given collision object.
		void BulletPhysicsData::RemoveCollisionObject(btCollisionObject *obj)
		{
			for (auto it = m_previousTickCollisions.begin(); it != m_previousTickCollisions.end(); )
			{
				if ((*it).first == obj || (*it).second == obj)
				{
					m_previousTickCollisions.erase(it++);
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
			BulletPhysicsData * const pBulletPhysics =
				static_cast<BulletPhysicsData*>(pWorld->getWorldUserInfo());
			pBulletPhysics->HandleCallback();
		}

		void BulletPhysicsData::HandleCallback()
		{
			CollisionPairs currentTickCollisions;
			HandleNewCollisions(currentTickCollisions);

			// Get collisions that existed on the last tick but not on this tick.
			CollisionPairs removedCollisions;
			std::set_difference(m_previousTickCollisions.begin(),
								m_previousTickCollisions.end(),
								currentTickCollisions.begin(), currentTickCollisions.end(),
								std::inserter(removedCollisions, removedCollisions.end()));

			std::for_each(removedCollisions.begin(), removedCollisions.end(),
				[this] (const CollisionPair& pair)
			{
				const btRigidBody * const body1 = pair.first;
				const btRigidBody * const body2 = pair.second;

				this->SendSeparationEvent(body1, body2);
			});

			m_previousTickCollisions = currentTickCollisions;
		}

		void BulletPhysicsData::HandleNewCollisions(CollisionPairs& currentTickCollisions)
		{
			for (int manifoldIdx = 0; manifoldIdx < m_pCollisionDispatcher->getNumManifolds(); ++manifoldIdx)
			{
				const btPersistentManifold * const pContactPoint =
					m_pCollisionDispatcher->getManifoldByIndexInternal(manifoldIdx);
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

				if (m_previousTickCollisions.find(newPair) == m_previousTickCollisions.end())
				{
					SendNewCollisionEvent(pContactPoint, body1, body2);
				}
			}
		}

		void BulletPhysicsData::AddShape(StrongActorPtr pActor, btCollisionShape *shape,
			float mass, const std::string& material)
		{
			assert(pActor.get());
			// There can be only one rigid body per (non-static) actor in this implementation.
			ActorID id = pActor->GetID();
			assert(m_actorToRigidBodyListMap.find(id) == m_actorToRigidBodyListMap.end());

			std::weak_ptr<WorldTransformComponent> pWeakWorldTransform
				= pActor->GetWorldTransform();
			if (!pWeakWorldTransform.expired())
			{
				std::shared_ptr<WorldTransformComponent> pWorldTransform(pWeakWorldTransform);
				btMotionState *motionState = GetMotionStateFrom(pWorldTransform);

				MaterialData matData(m_physicsMaterialData->LookupMaterial(material));

				btVector3 localInertia(0, 0, 0);
				if (mass > 0.f)
					shape->calculateLocalInertia(mass, localInertia);

				btRigidBody::btRigidBodyConstructionInfo rbInfo(
					mass, motionState, shape, localInertia);

				rbInfo.m_restitution = matData.m_restitution;
				rbInfo.m_friction = matData.m_friction;

				btRigidBody * const body = new btRigidBody(rbInfo);
				m_pDynamicsWorld->addRigidBody(body);

				m_actorToRigidBodyListMap[id].push_back(body);
				m_rigidBodyToActorMap[body] = id;
			}
		}

		void BulletPhysicsData::AddStaticColliderShape(StrongActorPtr pActor, btCollisionShape *shape, bool isTrigger)
		{
			assert(pActor.get());
			ActorID id = pActor->GetID();
			// Triggers may have only one rigid body but other static actors may have several.
			assert(!isTrigger || m_actorToRigidBodyListMap.find(id) == m_actorToRigidBodyListMap.end());

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

				if (isTrigger) // triggers may intersect with other objects (this causes a trigger related event, not a collision event)
				{
					// TODO: should I do this?
					// body->setUserPointer(pActor.get());
					body->setCollisionFlags(
						body->getCollisionFlags() | btRigidBody::CF_NO_CONTACT_RESPONSE);
				}

				m_actorToRigidBodyListMap[id].push_back(body);
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
			auto pGameData = GameData::getInstance();
			auto pEventManager = pGameData->GetEventManager();
			assert(pEventManager);

			std::shared_ptr<Events::IEventData> event;
			// Only triggers have a user pointer, so we use this
			// to distinguish them from other actors.
			if (pBody1->getUserPointer() || pBody2->getUserPointer())
			{
				bool firstIsTrigger = pBody1->getUserPointer() != nullptr;
				ActorID triggerId = firstIsTrigger ? GetActorID(pBody1) : GetActorID(pBody2);
				ActorID actorId = firstIsTrigger ? GetActorID(pBody2) : GetActorID(pBody1);
				event.reset(new Events::TriggerEntryEvent(pGameData->CurrentTimeSec(),
					triggerId, actorId));
				pEventManager->QueueEvent(event);
			}
			else // not a trigger event
			{
				ActorID id1 = GetActorID(pBody1);
				ActorID id2 = GetActorID(pBody2);
				if (id1 == 0 || id2 == 0)
					return;

				std::vector<Vec3> collisionPoints;
				btVector3 sumNormalForce;
				btVector3 sumFrictionForce;

				for (int i = 1; i < manifold->getNumContacts(); ++i)
				{
					const btManifoldPoint &point = manifold->getContactPoint(i);
					collisionPoints.push_back(btVector3_to_Vec3(point.getPositionWorldOnB()));

					sumNormalForce += point.m_combinedRestitution * point.m_normalWorldOnB;
					sumFrictionForce += point.m_combinedFriction * point.m_lateralFrictionDir1;
				}
				event.reset(new Events::ActorCollideEvent(pGameData->CurrentTimeSec(),
					id1, id2, collisionPoints, btVector3_to_Vec3(sumNormalForce),
					btVector3_to_Vec3(sumFrictionForce)));
				pEventManager->QueueEvent(event);
			}
		}

		void BulletPhysicsData::SendSeparationEvent(const btRigidBody * pBody1, const btRigidBody * pBody2)
		{
			auto pGameData = GameData::getInstance();
			auto pEventManager = pGameData->GetEventManager();
			assert(pEventManager);

			std::shared_ptr<Events::IEventData> event;
			if (pBody1->getUserPointer() || pBody2->getUserPointer())
			{
				bool firstIsTrigger = pBody1->getUserPointer() != nullptr;
				ActorID triggerId = firstIsTrigger ? GetActorID(pBody1) : GetActorID(pBody2);
				ActorID actorId = firstIsTrigger ? GetActorID(pBody2) : GetActorID(pBody1);
				event.reset(new Events::TriggerExitEvent(pGameData->CurrentTimeSec(),
					triggerId, actorId));
				pEventManager->QueueEvent(event);
			}
			else
			{
				ActorID id1 = GetActorID(pBody1);
				ActorID id2 = GetActorID(pBody2);
				if (id1 == 0 || id2 == 0)
					return;

				event.reset(new Events::ActorSeparationEvent(
					pGameData->CurrentTimeSec(), id1, id2));
				pEventManager->QueueEvent(event);
			}
		}
	}
}