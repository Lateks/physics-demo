#include "BulletPhysics.h"
#include "BulletPhysicsObject.h"
#include "BulletPhysicsConstraint.h"
#include "GameActor.h"
#include "WorldTransformComponent.h"
#include "GameData.h"
#include "IEventManager.h"
#include "XMLPhysicsData.h"
#include "Events.h"
#include "BulletConversions.h"
#include "BSPConverter.h"

#include <btBulletCollisionCommon.h>
#include <btBulletDynamicsCommon.h>
#include <LinearMath\btGeometryUtil.h>
#include <cassert>
#include <vector>
#include <memory>
#include <algorithm>
#include <cmath>
#include <set>
#include <stdexcept>
#include <iostream> // used for error output

using std::shared_ptr;
using std::weak_ptr;
using std::unique_ptr;

namespace GameEngine
{
	namespace Physics
	{
		typedef std::pair<const btRigidBody *, const btRigidBody *> CollisionPair;
		typedef std::set<CollisionPair> CollisionPairs;

		struct CollisionObject;

		struct BulletPhysicsData
		{
			// CONSTRUCTORS / DESTRUCTORS:
			BulletPhysicsData(float worldScale)
				: m_worldScaleConst(worldScale) { }
			virtual ~BulletPhysicsData();

			// MEMBERS:
			float m_worldScaleConst;

			// Bullet-related:
			unique_ptr<btDynamicsWorld> m_pDynamicsWorld;                   // - manages the other required components (declared below)
			unique_ptr<btBroadphaseInterface> m_pCollisionBroadPhase;       // - manages the first (rough) phase of collision detection
			unique_ptr<btCollisionDispatcher> m_pCollisionDispatcher;       // - manages the more accurate second phase of collision detection
			unique_ptr<btConstraintSolver> m_pConstraintSolver;             // - manages objects' freedom of motion
			unique_ptr<btDefaultCollisionConfiguration> m_pCollisionConfig; // - memory usage configuration

			unique_ptr<XMLPhysicsData> m_physicsMaterialData;

			/* Store the rigid bodies related to game actors.
			 * Several rigid bodies can be related to a single actor, but only
			 * a single actor may be related to any rigid body. At the moment
			 * only "static" actors (basically map elements) can own several
			 * rigid bodies.
			 */
			std::map<ActorID, std::shared_ptr<BulletPhysicsObject>> m_actorToBulletPhysicsObjectMap;
			std::map<const btRigidBody*, ActorID> m_rigidBodyToActorMap;
			std::map<IPhysicsEngine::PhysicsObjectType, int> m_collisionFlags;

			std::shared_ptr<BulletPhysicsObject> GetPhysicsObject(ActorID id) const;
			ActorID GetActorID(const btRigidBody *pBody) const;

			CollisionPairs m_previousTickCollisions;

			// METHODS:
			bool VInitializeSystems(const std::string& materialFileName);

			void AddShape(ActorPtr pActor, btCollisionShape *shape, CollisionObject& object);
			void AddSingleBodyShape(ActorPtr pActor, btCollisionShape *shape, CollisionObject& object);
			void AddMultiBodyShape(ActorPtr pActor, btCollisionShape *shape, CollisionObject& object);

			void SendNewCollisionEvent(const btPersistentManifold * manifold,
				const btRigidBody * pBody1, const btRigidBody * pBody2);
			void SendSeparationEvent(const btRigidBody * pBody1, const btRigidBody * pBody2);
			void RemoveCollisionObject(btCollisionObject *obj);
			void SetupSystems();
			void CleanUpRigidBodies();
			void HandleNewCollisions(CollisionPairs& currentTickCollisions);
			btMotionState *GetMotionStateFrom(const WorldTransformComponent& transform);
			void CreateRigidBody(ActorPtr pActor, btCollisionShape *shape, CollisionObject& object);

			// Bullet callback handling.
			static void BulletInternalTickCallback(btDynamicsWorld * const pWorld, const btScalar timeStep);
			void HandleCallback();
		};

		struct CollisionObject
		{
			CollisionObject(IPhysicsEngine::PhysicsObjectType type,
							const std::string& material,
							const std::string& density,
							const std::function<float()>& volumeCalculationStrategy,
							const std::function<float(float)>& rollingFrictionCalculationStrategy = [] (float friction) { return friction / 3.f; })
							: m_objectType(type),
							m_calculateVolume(volumeCalculationStrategy),
							m_calculateRollingFriction(rollingFrictionCalculationStrategy),
							m_material(material), m_density(density) { }

			const IPhysicsEngine::PhysicsObjectType m_objectType;
			const std::function<float()> m_calculateVolume;
			const std::function<float(float)> m_calculateRollingFriction;
			const std::string m_material;
			const std::string m_density;
		};

		/* The following functions can be used as volume calculation strategies
		 * for CollisionObject.
		 */
		inline float SphereVolume(float radius)
		{
			return (4.f/3.f) * 3.14159f * pow(radius, 3);
		}

		inline float BoxVolume(const btVector3& dimensions)
		{
			return dimensions.x() * dimensions.y() * dimensions.z();
		}

		// Calculates the axis-aligned bounding box of the given shape
		// and the volume for it.
		float AABBVolume(btCollisionShape *shape)
		{
			btVector3 aabbMin, aabbMax;
			shape->getAabb(btTransform::getIdentity(), aabbMin, aabbMax);

			const btVector3 dimensions = aabbMax - aabbMin;
			return BoxVolume(dimensions);
		}

		BulletPhysics::BulletPhysics(float worldScale) : m_pData(new BulletPhysicsData(worldScale)) { }

		BulletPhysics::~BulletPhysics() { }

		bool BulletPhysics::VInitEngine(const std::string& materialFileName)
		{
			return m_pData->VInitializeSystems(materialFileName);
		}

		void UpdateWorldTransform(ActorPtr pActor, const Vec3& pos, const Quaternion& rot)
		{
			shared_ptr<GameData> pGame = GameData::GetInstance();
			WorldTransformComponent& worldTrans = pActor->GetWorldTransform();

			bool changed = false;
			if (worldTrans.GetRotation() != rot)
			{
				worldTrans.SetRotation(rot);
				changed = true;
			}
			if (worldTrans.GetPosition() != pos)
			{
				worldTrans.SetPosition(pos);
				changed = true;
			}
			if (changed)
			{
				auto pEventManager = pGame->GetEventManager();
				if (pEventManager)
				{
					std::shared_ptr<Events::IEventData> event = std::make_shared<Events::ActorMoveEvent>(
						pGame->CurrentTimeMs(), pActor->GetID());
					pEventManager->VQueueEvent(event);
				}
			}
		}

		// Update the locations of all actors involved in the physics
		// simulation and signal changes in location with events.
		void BulletPhysics::VSyncScene()
		{
			std::shared_ptr<GameData> pGame = GameData::GetInstance();
			for (auto it = m_pData->m_actorToBulletPhysicsObjectMap.begin();
				it != m_pData->m_actorToBulletPhysicsObjectMap.end(); it++)
			{
				ActorID id = it->first;
				std::shared_ptr<BulletPhysicsObject> pObject = it->second;
				assert(pObject);
				if (pObject->GetNumBodies() > 1 || pObject->GetNumBodies() == 0)
					continue; // do not update static actors

				const btRigidBody *body = pObject->GetRigidBodies()[0];
				const Quaternion rot = btQuaternion_to_Quaternion(body->getOrientation());
				const Vec3 pos = btVector3_to_Vec3(body->getCenterOfMassPosition(), m_pData->m_worldScaleConst);

				auto pActor = pGame->GetActor(id);
				if (pActor)
				{
					UpdateWorldTransform(pActor, pos, rot);
				}
			}
		}

		void BulletPhysics::VUpdateSimulation(float deltaSec)
		{
			m_pData->m_pDynamicsWorld->stepSimulation(deltaSec, 4);
		}

		void BulletPhysics::VAddSphere(ActorPtr pActor, float radius,
			IPhysicsEngine::PhysicsObjectType type, const std::string& density,
			const std::string& material)
		{
			if (!pActor)
				return;

			float btRadius = m_pData->m_worldScaleConst * radius;
			btSphereShape * const sphereShape = new btSphereShape(btRadius);

			CollisionObject object(type, material, density,
				[btRadius] () { return SphereVolume(btRadius); },
				[] (float friction) { return friction / 5.f; });
			m_pData->AddShape(pActor, sphereShape, object);
		}

		void BulletPhysics::VAddBox(ActorPtr pActor, const Vec3& dimensions,
			IPhysicsEngine::PhysicsObjectType type, const std::string& density, const std::string& material)
		{
			if (!pActor)
				return;

			btVector3 btDimensions = Vec3_to_btVector3(dimensions, m_pData->m_worldScaleConst);
			btBoxShape * const boxShape = new btBoxShape(0.5f * btDimensions);

			CollisionObject object(type, material, density,
				[btDimensions] () { return BoxVolume(btDimensions); });
			m_pData->AddShape(pActor, boxShape, object);
		}

		void BulletPhysics::VAddConvexMesh(ActorPtr pActor, std::vector<Vec3>& vertices,
			IPhysicsEngine::PhysicsObjectType type, const std::string& density, const std::string& material)
		{
			if (!pActor)
				return;

			// Compute the convex hull of the given vertex cloud.
			btConvexHullShape * const convexShape = new btConvexHullShape();

			std::for_each(vertices.begin(), vertices.end(),
				[&convexShape, this] (Vec3& vertex) { convexShape->addPoint(
				Vec3_to_btVector3(vertex, this->m_pData->m_worldScaleConst)); });

			CollisionObject object(type, material, density,
				[convexShape] () { return AABBVolume(convexShape); });
			m_pData->AddShape(pActor, convexShape, object);
		}

		void BulletPhysics::VAddConvexMesh(ActorPtr pActor, std::vector<Vec4>& planeEquations,
			IPhysicsEngine::PhysicsObjectType type, const std::string& density, const std::string& material)
		{
			if (!pActor)
				return;

			btAlignedObjectArray<btVector3> vertices;
			btAlignedObjectArray<btVector3> btPlaneEquations;
			float scaling = m_pData->m_worldScaleConst;
			std::for_each(planeEquations.begin(), planeEquations.end(),
				[&btPlaneEquations, scaling] (Vec4& eq)
			{
				btVector3 btEq = Vec4_to_btVector3(eq);
				btEq[3] *= scaling;
				btPlaneEquations.push_back(btEq);
			});
			btGeometryUtil::getVerticesFromPlaneEquations(btPlaneEquations, vertices);

			btConvexHullShape *convexShape = new btConvexHullShape(&(vertices[0].getX()), vertices.size());

			CollisionObject object(type, material, density,
				[convexShape] () { return AABBVolume(convexShape); });
			m_pData->AddShape(pActor, convexShape, object);
		}

		void BulletPhysics::VLoadBspMap(BspLoader& bspLoad, ActorPtr pActor, const std::string& material)
		{
			if (!pActor)
				return;

			Utils::ConvertBsp(bspLoad,
				[this, pActor, &material] (std::vector<Vec4> planeEquations)
			{
				this->VAddConvexMesh(pActor, planeEquations,
					IPhysicsEngine::PhysicsObjectType::STATIC, "", material);
			});
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
		void BulletPhysics::VApplyForce(const Vec3& direction, float magnitude, ActorID id)
		{
			std::shared_ptr<BulletPhysicsObject> pObject = m_pData->GetPhysicsObject(id);
			assert(pObject);
			// Could e.g. log an error if the body is not found.
			if (pObject && !pObject->IsStatic() && pObject->GetNumBodies() > 0)
			{
				const btVector3 dir = Vec3_to_btVector3(direction).normalized();
				auto &bodies = pObject->GetRigidBodies();
				std::for_each(bodies.begin(), bodies.end(),
					[&dir, magnitude] (btRigidBody *pBody) {
						pBody->applyCentralImpulse(dir * magnitude);
				});
			}
		}

		void BulletPhysics::VApplyTorque(const Vec3& direction, float magnitude, ActorID id)
		{
			std::shared_ptr<BulletPhysicsObject> pObject = m_pData->GetPhysicsObject(id);
			assert(pObject);
			if (pObject && !pObject->IsStatic() && pObject->GetNumBodies() > 0)
			{
				const btVector3 dir = Vec3_to_btVector3(direction).normalized();
				auto &bodies = pObject->GetRigidBodies();
				std::for_each(bodies.begin(), bodies.end(),
					[&dir, magnitude] (btRigidBody *pBody) {
						pBody->applyTorqueImpulse(dir * magnitude);
				});
			}
		}

		void BulletPhysics::VStopActor(ActorID id)
		{
			VSetLinearVelocity(id, Vec3(1, 0, 0), 0.f);
			VSetAngularVelocity(id, Vec3(1, 0, 0), 0.f);
		}

		void BulletPhysics::VSetLinearVelocity(ActorID id, const Vec3& direction, float magnitude)
		{
			std::shared_ptr<BulletPhysicsObject> pObject = m_pData->GetPhysicsObject(id);
			assert(pObject);
			if (pObject && !pObject->IsStatic() && pObject->GetNumBodies() > 0)
			{
				const btVector3 dir = Vec3_to_btVector3(direction).normalized();
				float scaledMagnitude = m_pData->m_worldScaleConst * magnitude;
				auto &bodies = pObject->GetRigidBodies();
				std::for_each(bodies.begin(), bodies.end(),
					[&dir, scaledMagnitude] (btRigidBody *pBody) {
						pBody->setLinearVelocity(dir * scaledMagnitude);
				});
			}
		}

		void BulletPhysics::VSetAngularVelocity(ActorID id, const Vec3& rotationAxis, float radiansPerSecond)
		{
			std::shared_ptr<BulletPhysicsObject> pObject = m_pData->GetPhysicsObject(id);
			assert(pObject);
			if (pObject && !pObject->IsStatic() && pObject->GetNumBodies() > 0)
			{
				const btVector3 axis = Vec3_to_btVector3(rotationAxis).normalized();
				auto &bodies = pObject->GetRigidBodies();
				std::for_each(bodies.begin(), bodies.end(),
					[&axis, radiansPerSecond] (btRigidBody *pBody) {
						pBody->setAngularVelocity(axis * radiansPerSecond);
				});
			}
		}

		void BulletPhysics::VSetGlobalGravity(Vec3& gravity)
		{
			assert(m_pData && m_pData->m_pDynamicsWorld);
			m_pData->m_pDynamicsWorld->setGravity(Vec3_to_btVector3(gravity, m_pData->m_worldScaleConst));
		}

		Vec3 BulletPhysics::VGetGlobalGravity()
		{
			assert(m_pData && m_pData->m_pDynamicsWorld);
			return btVector3_to_Vec3(m_pData->m_pDynamicsWorld->getGravity(), m_pData->m_worldScaleConst);
		}

		ActorID BulletPhysics::VGetClosestActorHit(Vec3& rayFrom, Vec3& rayTo, Vec3& pickPosition) const
		{
			btVector3 btRayFrom = Vec3_to_btVector3(rayFrom, m_pData->m_worldScaleConst);
			btVector3 btRayTo = Vec3_to_btVector3(rayTo, m_pData->m_worldScaleConst);
			btCollisionWorld::AllHitsRayResultCallback rayCallback(btRayFrom, btRayTo);

			m_pData->m_pDynamicsWorld->rayTest(btRayFrom, btRayTo, rayCallback);

			ActorID actorHit = 0;
			float closestHitDistance = FLT_MAX;
			btVector3 btPickPosition;
			if (rayCallback.hasHit())
			{
				/* Pick the first body that was hit and was not a static or a kinematic object
				 * (those are unpickable). The collection of collision objects in the raycallback
				 * does not appear to be ordered by distance, so we need to keep track of the closest
				 * hit so far. Triggers are ignored and picking is not possible through walls or
				 * other static non-trigger objects.
				 *
				 * btCollisionWorld::ClosestRayResultCallback cannot be used here because we
				 * want to be able to pick objects that are inside trigger bodies, in which
				 * case the ClosestRayResultCallback would only detect a collision with the
				 * trigger body, which is unpickable.
				 */ 
				for (int i = 0; i < rayCallback.m_collisionObjects.size(); i++)
				{
					const btRigidBody *pBody = btRigidBody::upcast(rayCallback.m_collisionObjects[i]);
					assert(pBody);
					if (pBody)
					{
						auto actorId = m_pData->m_rigidBodyToActorMap[pBody];
						auto hitPosition = rayCallback.m_hitPointWorld[i];
						float objectDistance = (hitPosition-btRayFrom).length();
						if (pBody->isStaticOrKinematicObject())
						{
							if (!m_pData->GetPhysicsObject(actorId)->IsTrigger() && objectDistance < closestHitDistance)
							{
								actorHit = 0;
								closestHitDistance = objectDistance;
							}
						}
						else if (objectDistance < closestHitDistance)
						{
							actorHit = actorId;
							btPickPosition = hitPosition;
							closestHitDistance = objectDistance;
						}
					}
				}
			}
			if (actorHit)
			{
				pickPosition = btVector3_to_Vec3(btPickPosition, m_pData->m_worldScaleConst);
			}
			return actorHit;
		}

		inline btGeneric6DofConstraint *CreatePickConstraint(btRigidBody* pBody, btTransform *pTransform)
		{
			btGeneric6DofConstraint* dof6 = new btGeneric6DofConstraint(*pBody, *pTransform, false);
			dof6->setLinearLowerLimit(btVector3(0,0,0));
			dof6->setLinearUpperLimit(btVector3(0,0,0));
			dof6->setAngularLowerLimit(btVector3(0,0,0));
			dof6->setAngularUpperLimit(btVector3(0,0,0));

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

			return dof6;
		}

		/* Most of the constraint code is from Bullet tutorials (DemoApplication.cpp).
		 * This is really a convenience method to allow adding constraints for
		 * picking up items. In general, a physics SDK wrapper should possibly
		 * define more generic methods for setting up constraints (?).
		 */
		unsigned int BulletPhysics::VAddPickConstraint(ActorID actorId, Vec3& pickPosition, Vec3& cameraPosition)
		{
			std::shared_ptr<BulletPhysicsObject> pObject = m_pData->GetPhysicsObject(actorId);
			if (!pObject || pObject->IsStatic() || pObject->IsKinematic())
				return 0;

			// Calculate parameters for the constraint.
			btVector3 btPickPosition = Vec3_to_btVector3(pickPosition, m_pData->m_worldScaleConst);
			btVector3 btCameraPosition = Vec3_to_btVector3(cameraPosition, m_pData->m_worldScaleConst);
			float pickDistance = (btPickPosition - btCameraPosition).length();

			btRigidBody *pBody = pObject->GetRigidBodies()[0];
			pBody->setActivationState(DISABLE_DEACTIVATION);

			btVector3 localPivot = pBody->getCenterOfMassTransform().inverse() * btPickPosition;

			btTransform transform;
			transform.setIdentity();
			transform.setOrigin(localPivot);

			// Create the constraint and store it.
			btGeneric6DofConstraint* btPickConstraint = CreatePickConstraint(pBody, &transform);

			m_pData->m_pDynamicsWorld->addConstraint(btPickConstraint,true);
			ConstraintID pickConstraintId = pObject->AddConstraint(btPickConstraint,
				BulletPhysicsConstraint::ConstraintType::PICK_CONSTRAINT);

			// Create the event handler callback.
			Events::EventType handledType = Events::EventType::CAMERA_MOVED;
			Events::EventHandlerPtr pHandler(new std::function<void(Events::EventPtr)>(
				[this, pickConstraintId, actorId, handledType] (Events::EventPtr pEvent)
			{
				assert(pEvent->VGetEventType() == handledType);
				if (pEvent->VGetEventType() != handledType)
					return;

				std::shared_ptr<Events::CameraMoveEvent> pMoveEvent =
					std::dynamic_pointer_cast<Events::CameraMoveEvent>(pEvent);

				this->VUpdatePickConstraint(actorId, pickConstraintId,
					pMoveEvent->GetCameraPosition(), pMoveEvent->GetCameraTarget());
			})
			);

			std::shared_ptr<BulletPhysicsConstraint> pPickConstraint =
				pObject->GetConstraint(pickConstraintId);
			assert(pPickConstraint && pPickConstraint->GetBulletConstraint() &&
				pPickConstraint->GetConstraintType() == BulletPhysicsConstraint::ConstraintType::PICK_CONSTRAINT);

			// Store the callback function and picking distance for later reference.
			std::shared_ptr<BulletPickConstraint> pConstraint =
				std::dynamic_pointer_cast<BulletPickConstraint>(pPickConstraint);
			pConstraint->SetConstraintUpdater(pHandler, handledType);
			pConstraint->SetPickDistance(pickDistance);

			// Disable rotations to avoid jitter when rigid body is
			// pushed against e.g. a wall. Store the current angular
			// factor to restore it later. Set angular velocity to zero
			// to stop ongoing rotations at the moment of picking.
			pConstraint->SetOriginalAngularFactor(pBody->getAngularFactor());
			pBody->setAngularFactor(0);
			pBody->setAngularVelocity(btVector3(0, 0, 0));

			auto pGame = GameData::GetInstance();
			assert(pGame && pGame->GetEventManager());
			std::shared_ptr<Events::IEventManager> pEventMgr = pGame->GetEventManager();
			if (pEventMgr)
			{
				pEventMgr->VRegisterHandler(handledType, pHandler);
			}

			return pickConstraintId;
		}

		void BulletPhysics::VUpdatePickConstraint(ActorID actorId, ConstraintID constraintId, Vec3& rayFrom, Vec3& rayTo)
		{
			std::shared_ptr<BulletPhysicsObject> pObject = m_pData->GetPhysicsObject(actorId);
			if (!pObject || pObject->GetNumBodies() != 1 || pObject->GetNumConstraints() == 0)
				return;

			std::shared_ptr<BulletPhysicsConstraint> pPickConstraint = pObject->GetConstraint(constraintId);
			assert(pPickConstraint && pPickConstraint->GetBulletConstraint());

			bool isPickConstraint = pPickConstraint->GetConstraintType() ==
				BulletPhysicsConstraint::ConstraintType::PICK_CONSTRAINT;
			assert(isPickConstraint);
			if (!pPickConstraint || !pPickConstraint->GetBulletConstraint() || !isPickConstraint)
				return;

			std::shared_ptr<BulletPickConstraint> pConstraint =
				std::dynamic_pointer_cast<BulletPickConstraint>(pPickConstraint);

			// Update the pick constraint and keep it at the same picking distance as
			// when it was picked up.
			btTypedConstraint *btConstraint = pPickConstraint->GetBulletConstraint();
			if (btConstraint->getConstraintType() == D6_CONSTRAINT_TYPE)
			{
				btGeneric6DofConstraint* pDof6PickConstraint = static_cast<btGeneric6DofConstraint*>(btConstraint);
				if (pDof6PickConstraint)
				{
					btVector3 btRayFrom = Vec3_to_btVector3(rayFrom, m_pData->m_worldScaleConst);
					btVector3 btRayTo = Vec3_to_btVector3(rayTo, m_pData->m_worldScaleConst);

					btVector3 newPivot;
					btVector3 dir = btRayTo - btRayFrom;
					dir.normalize();
					dir *= pConstraint->GetPickDistance();

					newPivot = btRayFrom + dir;
					pDof6PickConstraint->getFrameOffsetA().setOrigin(newPivot);
				}
			}
		}

		void BulletPhysics::VRemoveConstraint(ActorID actorId, unsigned int constraintId)
		{
			std::shared_ptr<BulletPhysicsObject> pObject = m_pData->GetPhysicsObject(actorId);
			if (!pObject || pObject->GetNumBodies() != 1 || pObject->GetNumConstraints() == 0)
				return;
			std::shared_ptr<BulletPhysicsConstraint> pConstraint = pObject->GetConstraint(constraintId);
			assert(pConstraint && pConstraint->GetBulletConstraint());
			if (!pConstraint || !pConstraint->GetBulletConstraint())
				return;

			btRigidBody *pBody = pObject->GetRigidBodies()[0];
			m_pData->m_pDynamicsWorld->removeConstraint(pConstraint->GetBulletConstraint());

			// Restore angular factor to re-enable rotations.
			if (pConstraint->GetConstraintType() == BulletPhysicsConstraint::ConstraintType::PICK_CONSTRAINT)
			{
				std::shared_ptr<BulletPickConstraint> pPickConstraint =
					std::dynamic_pointer_cast<BulletPickConstraint>(pConstraint);
				pBody->setAngularFactor(pPickConstraint->GetOriginalAngularFactor());
			}

			pBody->forceActivationState(ACTIVE_TAG);
			pBody->setDeactivationTime(0.f);

			auto pGame = GameData::GetInstance();
			assert(pGame && pGame->GetEventManager());
			std::shared_ptr<Events::IEventManager> pEventMgr = pGame->GetEventManager();
			if (pEventMgr)
			{
				pEventMgr->VDeregisterHandler(pConstraint->GetHandlerEventType(), pConstraint->GetConstraintUpdater());
			}
			pObject->RemoveConstraint(constraintId);
		}

		std::shared_ptr<BulletPhysicsObject> BulletPhysicsData::GetPhysicsObject(ActorID id) const
		{
			auto it = m_actorToBulletPhysicsObjectMap.find(id);
			if (it != m_actorToBulletPhysicsObjectMap.end())
				return it->second;
			return std::shared_ptr<BulletPhysicsObject>();
		}

		ActorID BulletPhysicsData::GetActorID(const btRigidBody *pBody) const
		{
			auto it = m_rigidBodyToActorMap.find(pBody);
			assert(it != m_rigidBodyToActorMap.end());
			if (it != m_rigidBodyToActorMap.end())
				return it->second;
			return 0;
		}

		bool BulletPhysicsData::VInitializeSystems(const std::string& materialFileName)
		{
			m_physicsMaterialData.reset(new XMLPhysicsData());
			if (!m_physicsMaterialData->LoadDataFromXML(materialFileName))
			{
				return false; // XML parsing failed
			}

			SetupSystems();

			if (!m_pCollisionConfig || !m_pCollisionDispatcher ||
				!m_pCollisionBroadPhase || !m_pConstraintSolver ||
				!m_pDynamicsWorld)
			{
				return false;
			}

			m_pDynamicsWorld->setInternalTickCallback(BulletInternalTickCallback);
			m_pDynamicsWorld->setWorldUserInfo(this);

			m_collisionFlags[IPhysicsEngine::PhysicsObjectType::DYNAMIC] = 0;
			m_collisionFlags[IPhysicsEngine::PhysicsObjectType::STATIC] = btRigidBody::CF_STATIC_OBJECT;
			m_collisionFlags[IPhysicsEngine::PhysicsObjectType::TRIGGER] =
				btRigidBody::CF_STATIC_OBJECT | btRigidBody::CF_NO_CONTACT_RESPONSE;

			return true;
		}

		BulletPhysicsData::~BulletPhysicsData()
		{
			CleanUpRigidBodies();
		}

		void BulletPhysicsData::SetupSystems()
		{
			m_pCollisionConfig.reset(new btDefaultCollisionConfiguration());
			m_pCollisionDispatcher.reset(new btCollisionDispatcher(m_pCollisionConfig.get()));
			m_pCollisionBroadPhase.reset(new btDbvtBroadphase());
			m_pConstraintSolver.reset(new btSequentialImpulseConstraintSolver());

			m_pDynamicsWorld.reset(new btDiscreteDynamicsWorld(
				m_pCollisionDispatcher.get(), m_pCollisionBroadPhase.get(),
				m_pConstraintSolver.get(), m_pCollisionConfig.get()));
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
		}

		void BulletPhysicsData::RemoveCollisionObject(btCollisionObject *obj)
		{
			// Remove the collision pairs that contain the given collision object.
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

				// destroy related constraints
				for (int i = body->getNumConstraintRefs()-1; i >= 0; i--)
				{
					btTypedConstraint *pConstraint = body->getConstraintRef(i);
					m_pDynamicsWorld->removeConstraint(pConstraint);
					delete pConstraint;
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
					static_cast<const btRigidBody *>(pContactPoint->getBody0());
				const btRigidBody * body2 =
					static_cast<const btRigidBody *>(pContactPoint->getBody1());

				// Keep rigid body pointers always in the same order to make
				// comparisons between collision pairs easier.
				if (body1 > body2)
				{
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

		void BulletPhysicsData::AddShape(ActorPtr pActor, btCollisionShape *shape, CollisionObject& object)
		{
			assert (pActor);
			if (!pActor)
			{
				delete shape;
				return;
			}

			switch (object.m_objectType)
			{
			case IPhysicsEngine::PhysicsObjectType::DYNAMIC:
				AddSingleBodyShape(pActor, shape, object);
				break;
			case IPhysicsEngine::PhysicsObjectType::STATIC:
				AddMultiBodyShape(pActor, shape, object);
				break;
			case IPhysicsEngine::PhysicsObjectType::TRIGGER:
				AddSingleBodyShape(pActor, shape, object);
				break;
			default: // Note: Kinematic objects are not supported.
				delete shape;
				throw std::domain_error("Unsupported physics object type given.");
			}
		}

		void BulletPhysicsData::AddSingleBodyShape(ActorPtr pActor, btCollisionShape *shape, CollisionObject& object)
		{
			ActorID id = pActor->GetID();
			auto iter = m_actorToBulletPhysicsObjectMap.find(id);
			assert(iter == m_actorToBulletPhysicsObjectMap.end());
			if (iter != m_actorToBulletPhysicsObjectMap.end())
			{
				auto &rigidBodies = iter->second->GetRigidBodies();
				std::for_each(rigidBodies.begin(), rigidBodies.end(),
					[this] (btRigidBody *pBody)
				{
					m_rigidBodyToActorMap.erase(pBody);
					RemoveCollisionObject(pBody);
				});
			}
			m_actorToBulletPhysicsObjectMap[id].reset(new BulletPhysicsObject(id, object.m_objectType));

			CreateRigidBody(pActor, shape, object);
		}

		// Note: (currently) in this implementation, only static non-trigger objects may have
		// several rigid bodies.
		void BulletPhysicsData::AddMultiBodyShape(ActorPtr pActor, btCollisionShape *shape, CollisionObject& object)
		{
			assert(object.m_objectType == IPhysicsEngine::PhysicsObjectType::STATIC);
			ActorID id = pActor->GetID();

			auto objectIt = m_actorToBulletPhysicsObjectMap.find(id);
			if (objectIt == m_actorToBulletPhysicsObjectMap.end())
			{
				m_actorToBulletPhysicsObjectMap[id].reset(
					new BulletPhysicsObject(id, object.m_objectType));
			}

			CreateRigidBody(pActor, shape, object);
		}

		void BulletPhysicsData::CreateRigidBody(ActorPtr pActor, btCollisionShape *shape, CollisionObject& object)
		{
			static MaterialData defaultMaterial = MaterialData(0.f, 0.f);
			const MaterialData& matData = object.m_material.empty() ?
				defaultMaterial : m_physicsMaterialData->LookupMaterial(object.m_material);
			float density = object.m_density.empty() ?
				0.f : m_physicsMaterialData->LookupDensity(object.m_density);
			float mass = density > 0.f ? object.m_calculateVolume() * density : 0.f;

			btMotionState *motionState = GetMotionStateFrom(pActor->GetWorldTransform());
			btVector3 localInertia(0, 0, 0);
			if (mass > 0.f)
				shape->calculateLocalInertia(mass, localInertia);

			btRigidBody::btRigidBodyConstructionInfo rbInfo(
				mass, motionState, shape, localInertia);

			rbInfo.m_restitution = matData.m_restitution;
			rbInfo.m_friction = matData.m_friction; // this is the sliding friction (as opposed to rolling friction)
			rbInfo.m_rollingFriction = object.m_calculateRollingFriction(matData.m_friction);

			btRigidBody * const pBody = new btRigidBody(rbInfo);
			if (pBody->getRollingFriction() > 0.f)
			{
				pBody->setAnisotropicFriction(shape->getAnisotropicRollingFrictionDirection(),
					btCollisionObject::CF_ANISOTROPIC_ROLLING_FRICTION);
			}

			pBody->setCollisionFlags(pBody->getCollisionFlags() | m_collisionFlags[object.m_objectType]);

			m_pDynamicsWorld->addRigidBody(pBody);

			ActorID id = pActor->GetID();
			m_actorToBulletPhysicsObjectMap[id]->AddRigidBody(pBody);
			m_rigidBodyToActorMap[pBody] = id;
		}

		btMotionState *BulletPhysicsData::GetMotionStateFrom(const WorldTransformComponent& worldTransform)
		{
			btQuaternion rotation(Quaternion_to_btQuaternion(worldTransform.GetRotation()));
			btVector3 translation(Vec3_to_btVector3(worldTransform.GetPosition(), m_worldScaleConst));
			btTransform transform(rotation, translation);
			return new btDefaultMotionState(transform);
		}

		void BulletPhysicsData::SendNewCollisionEvent(const btPersistentManifold * manifold,
			const btRigidBody * pBody1, const btRigidBody * pBody2)
		{
			auto pGameData = GameData::GetInstance();
			auto pEventManager = pGameData->GetEventManager();
			assert(pEventManager);

			ActorID id1 = GetActorID(pBody1);
			ActorID id2 = GetActorID(pBody2);
			if (id1 == 0 || id2 == 0)
				return;

			std::shared_ptr<Events::IEventData> event;
			bool firstIsTrigger;
			if ((firstIsTrigger = m_actorToBulletPhysicsObjectMap[id1]->IsTrigger()) ||
				m_actorToBulletPhysicsObjectMap[id2]->IsTrigger())
			{
				ActorID triggerId = firstIsTrigger ? GetActorID(pBody1) : GetActorID(pBody2);
				ActorID actorId = firstIsTrigger ? GetActorID(pBody2) : GetActorID(pBody1);
				event.reset(new Events::TriggerEntryEvent(pGameData->CurrentTimeMs(),
					triggerId, actorId));
				pEventManager->VQueueEvent(event);
			}
			else // not a trigger event
			{
				std::vector<Vec3> collisionPoints;
				btVector3 sumNormalForce;
				btVector3 sumFrictionForce;

				for (int i = 1; i < manifold->getNumContacts(); ++i)
				{
					const btManifoldPoint &point = manifold->getContactPoint(i);
					collisionPoints.push_back(btVector3_to_Vec3(point.getPositionWorldOnB(), m_worldScaleConst));

					sumNormalForce += point.m_combinedRestitution * point.m_normalWorldOnB;
					sumFrictionForce += point.m_combinedFriction * point.m_lateralFrictionDir1;
				}
				event.reset(new Events::ActorCollideEvent(pGameData->CurrentTimeMs(),
					id1, id2, collisionPoints, btVector3_to_Vec3(sumNormalForce, m_worldScaleConst),
					btVector3_to_Vec3(sumFrictionForce, m_worldScaleConst)));
				pEventManager->VQueueEvent(event);
			}
		}

		void BulletPhysicsData::SendSeparationEvent(const btRigidBody * pBody1, const btRigidBody * pBody2)
		{
			auto pGameData = GameData::GetInstance();
			auto pEventManager = pGameData->GetEventManager();
			assert(pEventManager);

			ActorID id1 = GetActorID(pBody1);
			ActorID id2 = GetActorID(pBody2);
			if (id1 == 0 || id2 == 0)
				return;

			std::shared_ptr<Events::IEventData> event;
			bool firstIsTrigger;
			if ((firstIsTrigger = m_actorToBulletPhysicsObjectMap[id1]->IsTrigger()) ||
				m_actorToBulletPhysicsObjectMap[id2]->IsTrigger())
			{
				ActorID triggerId = firstIsTrigger ? GetActorID(pBody1) : GetActorID(pBody2);
				ActorID actorId = firstIsTrigger ? GetActorID(pBody2) : GetActorID(pBody1);
				event.reset(new Events::TriggerExitEvent(pGameData->CurrentTimeMs(),
					triggerId, actorId));
				pEventManager->VQueueEvent(event);
			}
			else
			{
				event.reset(new Events::ActorSeparationEvent(
					pGameData->CurrentTimeMs(), id1, id2));
				pEventManager->VQueueEvent(event);
			}
		}
	}
}