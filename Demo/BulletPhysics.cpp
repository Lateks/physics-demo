#include "BulletPhysics.h"
#include "BulletPhysicsData.h"
#include "BulletPhysicsObject.h"
#include "GameActor.h"
#include "WorldTransformComponent.h"
#include "GameData.h"
#include "IEventManager.h"
#include "XMLPhysicsData.h"
#include "Events.h"
#include "BulletConversions.h"
#include "BSPConverter.h"
#include "utils.h"

#include <btBulletCollisionCommon.h>
#include <btBulletDynamicsCommon.h>
#include <LinearMath\btGeometryUtil.h>
#include <cassert>
#include <vector>
#include <memory>
#include <algorithm>

using std::shared_ptr;
using std::weak_ptr;

namespace GameEngine
{
	namespace Physics
	{
		BulletPhysics::BulletPhysics() : m_pData(new BulletPhysicsData()) { }

		BulletPhysics::~BulletPhysics()
		{
			delete m_pData;
		}

		bool BulletPhysics::VInitEngine()
		{
			return m_pData->VInitializeSystems();
		}

		// Update the locations of all actors involved in the physics
		// simulation and signal changes in location with events.
		void BulletPhysics::VSyncScene()
		{
			GameData *game = GameData::getInstance();
			for (auto it = m_pData->m_actorToBulletPhysicsObjectMap.begin();
				it != m_pData->m_actorToBulletPhysicsObjectMap.end(); it++)
			{
				ActorID id = it->first;
				std::shared_ptr<BulletPhysicsObject> pObject = it->second;
				assert(pObject.get());
				if (pObject->GetNumBodies() > 1 || pObject->GetNumBodies() == 0)
					continue; // do not update static actors

				const btRigidBody *body = pObject->GetRigidBodies()[0];
				const Quaternion rot = btQuaternion_to_Quaternion(body->getOrientation());
				const Vec3 pos = btVector3_to_Vec3(body->getCenterOfMassPosition());

				WeakActorPtr pWeakActor = game->GetActor(id);
				if (!pWeakActor.expired())
				{
					StrongActorPtr pActor(pWeakActor);
					weak_ptr<WorldTransformComponent> pWeakWorldTrans =
						pActor->GetWorldTransform();

					if (!pWeakWorldTrans.expired())
					{
						bool changed = false;
						shared_ptr<WorldTransformComponent> pWorldTrans(pWeakWorldTrans);
						if (pWorldTrans->GetRotation() != rot)
						{
							pWorldTrans->SetRotation(rot);
							changed = true;
						}
						if (pWorldTrans->GetPosition() != pos)
						{
							pWorldTrans->SetPosition(pos);
							changed = true;
						}
						if (changed)
						{
							Events::EventPtr event;
							event.reset(new Events::ActorMoveEvent(game->CurrentTimeSec(), id));
							game->GetEventManager()->QueueEvent(event);
						}
					}
				}
			}
		}

		void BulletPhysics::VUpdateSimulation(float deltaSec)
		{
			m_pData->m_pDynamicsWorld->stepSimulation(deltaSec, 4);
		}

		void BulletPhysics::VAddSphere(float radius, WeakActorPtr pActor,
			const std::string& density, const std::string& material)
		{
			if (pActor.expired())
				return;
			StrongActorPtr pStrongActor(pActor);

			btSphereShape * const collisionShape = new btSphereShape(radius);

			float matDensity = m_pData->m_physicsMaterialData->LookupDensity(density);
			const float sphereVolume = (4.f/3.f) * 3.14159f * radius * radius * radius;
			const float mass = sphereVolume * matDensity;

			m_pData->AddShape(pStrongActor, collisionShape, mass, material);
		}

		void BulletPhysics::VAddBox(const Vec3& dimensions, WeakActorPtr pActor,
			const std::string& density, const std::string& material)
		{
			if (pActor.expired())
				return;
			StrongActorPtr pStrongActor(pActor);

			Vec3 boxHalfExtents = 0.5f * dimensions;
			btBoxShape * const boxShape = new btBoxShape(Vec3_to_btVector3(boxHalfExtents));

			float matDensity = m_pData->m_physicsMaterialData->LookupDensity(density);
			const float boxVolume = dimensions.x() * dimensions.y() * dimensions.z();
			const float mass = boxVolume * matDensity;

			m_pData->AddShape(pStrongActor, boxShape, mass, material);
		}

		void BulletPhysics::VAddConvexMesh(std::vector<Vec3>& vertices,
				WeakActorPtr pActor, const std::string& density, const std::string& material)
		{
			if (pActor.expired())
				return;
			StrongActorPtr pStrongActor(pActor);

			btConvexHullShape * const convexShape = new btConvexHullShape();

			std::for_each(vertices.begin(), vertices.end(),
				[&convexShape] (Vec3& vertex) { convexShape->addPoint(Vec3_to_btVector3(vertex)); });

			// Approximate mass using an axis-aligned bounding box.
			btVector3 aabbMin, aabbMax;
			convexShape->getAabb(btTransform::getIdentity(), aabbMin, aabbMax);

			const btVector3 dimensions = aabbMax - aabbMin;
			const float matDensity = m_pData->m_physicsMaterialData->LookupDensity(density);
			const float aabbVolume = dimensions.x() * dimensions.y() * dimensions.z();
			const float mass = aabbVolume * matDensity;

			m_pData->AddShape(pStrongActor, convexShape, mass, material);
		}

		void BulletPhysics::VAddConvexStaticColliderMesh(std::vector<Vec3>& vertices, WeakActorPtr pActor)
		{
			if (pActor.expired())
				return;
			StrongActorPtr pStrongActor(pActor);

			btConvexHullShape* convexShape = new btConvexHullShape();
			std::for_each(vertices.begin(), vertices.end(),
				[&convexShape] (Vec3& vertex) { convexShape->addPoint(Vec3_to_btVector3(vertex)); });
			m_pData->AddStaticColliderShape(pStrongActor, convexShape);
		}

		void BulletPhysics::VAddConvexStaticColliderMesh(std::vector<Vec4>& planeEquations, WeakActorPtr pActor)
		{
			if (pActor.expired())
				return;
			StrongActorPtr pStrongActor(pActor);

			btAlignedObjectArray<btVector3> vertices;
			btAlignedObjectArray<btVector3> btPlaneEquations;
			std::for_each(planeEquations.begin(), planeEquations.end(),
				[&btPlaneEquations] (Vec4& eq)
			{ btPlaneEquations.push_back(Vec4_to_btVector3(eq)); });
			btGeometryUtil::getVerticesFromPlaneEquations(btPlaneEquations, vertices);

			btConvexHullShape *convexShape = new btConvexHullShape(&(vertices[0].getX()), vertices.size());
			m_pData->AddStaticColliderShape(pStrongActor, convexShape);
		}

		void BulletPhysics::VLoadBspMap(BspLoader& bspLoad, WeakActorPtr pActor)
		{
			if (pActor.expired())
				return;

			BspConverter bspConv;
			bspConv.convertBsp(bspLoad,
				[this, pActor] (std::vector<Vec4> planeEquations)
			{
				this->VAddConvexStaticColliderMesh(planeEquations, pActor);
			});
		}

		void BulletPhysics::VCreateTrigger(WeakActorPtr pActor, const float dim)
		{
			if (pActor.expired())
				return;
			StrongActorPtr pStrongActor(pActor);

			// Create a cube-shaped trigger area. Of course, this could really
			// be any convex shape. The common functionality associated with
			// adding immovable colliders or triggers of any shape is in
			// BulletPhysicsData::AddStaticColliderShape.
			btBoxShape * const boxShape =
				new btBoxShape(Vec3_to_btVector3(Vec3(dim, dim, dim)));
			m_pData->AddStaticColliderShape(pStrongActor, boxShape, true);
		}
		
		void BulletPhysics::VRemoveActor(ActorID id)
		{
			std::vector<btRigidBody*> bodies = m_pData->GetPhysicsObject(id)->GetRigidBodies();
			if (bodies.size() > 0)
			{
				std::for_each(bodies.begin(), bodies.end(),
					[this] (btRigidBody *body)
				{
					this->m_pData->RemoveCollisionObject(body);
					this->m_pData->m_rigidBodyToActorMap.erase(body);
				});
				m_pData->m_actorToBulletPhysicsObjectMap.erase(id);
			}
		}

		/* VApplyForce and VApplyTorque are presented in the Game Coding Complete
		 * example code, but I could not figure out good uses for these. Forces
		 * are cleared out after each simulation step, so either the impulse
		 * applied to the object must be very strong or the force must be
		 * applied repeatedly for several simulation steps for it to have an
		 * effect.
		 */
		void BulletPhysics::VApplyForce(const Vec3& direction, float newtons, ActorID id)
		{
			std::shared_ptr<BulletPhysicsObject> pObject = m_pData->GetPhysicsObject(id);
			// Could e.g. log an error if the body is not found.
			if (pObject->GetNumBodies() > 0)
			{
				assert(pObject->GetNumBodies() == 1); // only static actors can have many bodies
				const btVector3 dir = Vec3_to_btVector3(direction);
				pObject->GetRigidBodies()[0]->applyCentralImpulse(dir.normalized() * newtons);
			}
		}

		void BulletPhysics::VApplyTorque(const Vec3& direction, float magnitude, ActorID id)
		{
			std::shared_ptr<BulletPhysicsObject> pObject = m_pData->GetPhysicsObject(id);
			if (pObject->GetNumBodies() > 0)
			{
				assert(pObject->GetNumBodies() == 1);
				const btVector3 dir = Vec3_to_btVector3(direction);
				pObject->GetRigidBodies()[0]->applyTorqueImpulse(dir.normalized() * magnitude);
			}
		}

		void BulletPhysics::VStopActor(ActorID id)
		{
			VSetLinearVelocity(id, Vec3(0, 0, 0), 1.f);
			VSetAngularVelocity(id, Vec3(0, 0, 0), 1.f);
		}

		void BulletPhysics::VSetLinearVelocity(ActorID id, const Vec3& direction, float magnitude)
		{
			std::shared_ptr<BulletPhysicsObject> pObject = m_pData->GetPhysicsObject(id);
			if (pObject->GetNumBodies() > 0)
			{
				assert(pObject->GetNumBodies() == 1);

				const btVector3 dir = Vec3_to_btVector3(direction).normalized();
				pObject->GetRigidBodies()[0]->setLinearVelocity(dir * magnitude);
			}
		}

		void BulletPhysics::VSetAngularVelocity(ActorID id, const Vec3& rotationAxis, float magnitude)
		{
			std::shared_ptr<BulletPhysicsObject> pObject = m_pData->GetPhysicsObject(id);
			if (pObject->GetNumBodies() > 0)
			{
				assert(pObject->GetNumBodies() == 1);

				const btVector3 axis = Vec3_to_btVector3(rotationAxis).normalized();
				pObject->GetRigidBodies()[0]->setAngularVelocity(axis * magnitude);
			}
		}

		void BulletPhysics::VSetGlobalGravity(Vec3& gravity)
		{
			assert(m_pData && m_pData->m_pDynamicsWorld);
			m_pData->m_pDynamicsWorld->setGravity(Vec3_to_btVector3(gravity));
		}

		ActorID BulletPhysics::GetClosestActorHit(Vec3& rayFrom, Vec3& rayTo, Vec3& pickPosition) const
		{
			btVector3 btRayFrom = Vec3_to_btVector3(rayFrom);
			btVector3 btRayTo = Vec3_to_btVector3(rayTo);
			btCollisionWorld::ClosestRayResultCallback rayCallback(btRayFrom, btRayTo);

			m_pData->m_pDynamicsWorld->rayTest(btRayFrom, btRayTo, rayCallback);
			ActorID actorHit = 0;
			if (rayCallback.hasHit())
			{
				const btRigidBody *pBody = btRigidBody::upcast(rayCallback.m_collisionObject);
				assert(pBody);
				if (pBody && !pBody->isStaticOrKinematicObject())
				{
					actorHit = m_pData->m_rigidBodyToActorMap[pBody];
					pickPosition = btVector3_to_Vec3(rayCallback.m_hitPointWorld);
				}
			}
			return actorHit;
		}

		/* Constraint code from Bullet tutorials (DemoApplication.cpp).
		 * This is really a convenience method to allow adding constraints for
		 * picking up items. In general, a physics SDK wrapper should possibly
		 * define more generic methods for setting up constraints (?).
		 */
		unsigned int BulletPhysics::AddPickConstraint(ActorID actorId, Vec3& pickPosition)
		{
			std::shared_ptr<BulletPhysicsObject> pObject = m_pData->GetPhysicsObject(actorId);
			if (!pObject.get() || pObject->IsStatic() || pObject->IsKinematic())
				return 0;

			btVector3 btPickPosition = Vec3_to_btVector3(pickPosition);
			btRigidBody *pBody = pObject->GetRigidBodies()[0];
			pBody->setActivationState(DISABLE_DEACTIVATION);

			btVector3 localPivot = pBody->getCenterOfMassTransform().inverse() * btPickPosition;

			btTransform transform;
			transform.setIdentity();
			transform.setOrigin(localPivot);

			btGeneric6DofConstraint* dof6 = new btGeneric6DofConstraint(*pBody, transform, false);
			dof6->setLinearLowerLimit(btVector3(0,0,0));
			dof6->setLinearUpperLimit(btVector3(0,0,0));
			dof6->setAngularLowerLimit(btVector3(0,0,0));
			dof6->setAngularUpperLimit(btVector3(0,0,0));

			m_pData->m_pDynamicsWorld->addConstraint(dof6,true);
			ConstraintID pickConstraintId = pObject->AddConstraint(dof6);

			dof6->setParam(BT_CONSTRAINT_STOP_CFM,0.8f,0);
			dof6->setParam(BT_CONSTRAINT_STOP_CFM,0.8f,1);
			dof6->setParam(BT_CONSTRAINT_STOP_CFM,0.8f,2);
			dof6->setParam(BT_CONSTRAINT_STOP_CFM,0.8f,3);
			dof6->setParam(BT_CONSTRAINT_STOP_CFM,0.8f,4);
			dof6->setParam(BT_CONSTRAINT_STOP_CFM,0.8f,5);

			dof6->setParam(BT_CONSTRAINT_STOP_ERP,0.1f,0);
			dof6->setParam(BT_CONSTRAINT_STOP_ERP,0.1f,1);
			dof6->setParam(BT_CONSTRAINT_STOP_ERP,0.1f,2);
			dof6->setParam(BT_CONSTRAINT_STOP_ERP,0.1f,3);
			dof6->setParam(BT_CONSTRAINT_STOP_ERP,0.1f,4);
			dof6->setParam(BT_CONSTRAINT_STOP_ERP,0.1f,5);

			Events::EventType handledType = Events::EventType::CAMERA_MOVED;
			Events::EventHandlerPtr pHandler(new std::function<void(Events::EventPtr)>(
				[this, pickConstraintId, actorId, handledType] (Events::EventPtr pEvent)
			{
				assert(pEvent->GetEventType() == handledType);
				if (pEvent->GetEventType() != handledType)
					return;

				Events::CameraMoveEvent *pMoveEvent =
					dynamic_cast<Events::CameraMoveEvent*>(pEvent.get());

				this->UpdatePickConstraint(actorId, pickConstraintId,
					pMoveEvent->GetRayFrom(), pMoveEvent->GetRayTo());
			})
			);

			std::shared_ptr<BulletPhysicsConstraint> pPickConstraint =
				pObject->GetConstraint(pickConstraintId);
			assert(pPickConstraint.get() && pPickConstraint->GetBulletConstraint());
			pPickConstraint->SetConstraintUpdater(pHandler, handledType);

			GameData *pGame = GameData::getInstance();
			assert(pGame && pGame->GetEventManager());
			Events::IEventManager *pEventMgr = pGame->GetEventManager();
			if (pEventMgr)
			{
				pEventMgr->RegisterHandler(handledType, pHandler);
			}

			return pickConstraintId;
		}

		void BulletPhysics::UpdatePickConstraint(ActorID actorId, ConstraintID constraintId, Vec3& rayFrom, Vec3& rayTo)
		{
			std::shared_ptr<BulletPhysicsObject> pObject = m_pData->GetPhysicsObject(actorId);
			if (!pObject.get() || pObject->GetNumBodies() != 1 || pObject->GetNumConstraints() == 0)
				return;

			std::shared_ptr<BulletPhysicsConstraint> pPickConstraint = pObject->GetConstraint(constraintId);
			assert(pPickConstraint.get() && pPickConstraint->GetBulletConstraint());
			if (!pPickConstraint.get() || !pPickConstraint->GetBulletConstraint())
				return;

			btTypedConstraint *btConstraint = pPickConstraint->GetBulletConstraint();
			if (btConstraint->getConstraintType() == D6_CONSTRAINT_TYPE)
			{
				btGeneric6DofConstraint* pDof6PickConstraint = static_cast<btGeneric6DofConstraint*>(btConstraint);
				if (pDof6PickConstraint)
				{
					btVector3 btRayFrom = Vec3_to_btVector3(rayFrom);
					btVector3 btRayTo = Vec3_to_btVector3(rayTo);

					btVector3 newPivot;
					btVector3 dir = btRayTo - btRayFrom;
					// TODO:
					// dir.normalize();
					// dir *= gOldPickingDist;

					newPivot = btRayFrom + dir;
					pDof6PickConstraint->getFrameOffsetA().setOrigin(newPivot);
				}
			}
		}

		void BulletPhysics::RemoveConstraint(ActorID actorId, unsigned int constraintId)
		{
			std::shared_ptr<BulletPhysicsObject> pObject = m_pData->GetPhysicsObject(actorId);
			if (!pObject.get() || pObject->GetNumBodies() != 1 || pObject->GetNumConstraints() == 0)
				return;
			std::shared_ptr<BulletPhysicsConstraint> pPickConstraint = pObject->GetConstraint(constraintId);
			assert(pPickConstraint && pPickConstraint->GetBulletConstraint());
			if (!pPickConstraint || !pPickConstraint->GetBulletConstraint())
				return;

			btRigidBody *pBody = pObject->GetRigidBodies()[0];
			m_pData->m_pDynamicsWorld->removeConstraint(pPickConstraint->GetBulletConstraint());

			pBody->forceActivationState(ACTIVE_TAG);
			pBody->setDeactivationTime(0.f);

			GameData *pGame = GameData::getInstance();
			assert(pGame && pGame->GetEventManager());
			Events::IEventManager *pEventMgr = pGame->GetEventManager();
			if (pEventMgr)
			{
				pEventMgr->DeregisterHandler(pPickConstraint->GetHandlerEventType(), pPickConstraint->GetConstraintUpdater());
			}
			pObject->RemoveConstraint(constraintId);
		}
	}
}