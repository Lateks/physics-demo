#include "BulletPhysics.h"
#include "BulletPhysicsData.h"
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
			for (auto it = m_pData->m_actorToRigidBodyListMap.begin();
				      it != m_pData->m_actorToRigidBodyListMap.end(); it++)
			{
				ActorID id = it->first;
				std::vector<btRigidBody*> bodies = it->second;
				if (bodies.size() > 1)
					continue; // do not update static actors

				const btRigidBody *body = bodies[0];
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

			btBoxShape * const boxShape = new btBoxShape(Vec3_to_btVector3(dimensions));

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

		void BulletPhysics::VAddBspMap(BspLoader& bspLoad, WeakActorPtr pActor)
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
			std::vector<btRigidBody*> bodies = m_pData->GetRigidBodies(id);
			if (bodies.size() > 0)
			{
				std::for_each(bodies.begin(), bodies.end(),
					[this] (btRigidBody *body)
				{
					this->m_pData->RemoveCollisionObject(body);
					this->m_pData->m_rigidBodyToActorMap.erase(body);
				});
				m_pData->m_actorToRigidBodyListMap.erase(id);
			}
		}

		void BulletPhysics::VApplyForce(const Vec3& direction, float newtons, ActorID id)
		{
			std::vector<btRigidBody*> bodies = m_pData->GetRigidBodies(id);
			// Could e.g. log an error if the body is not found.
			if (bodies.size() > 0)
			{
				assert(bodies.size() == 1); // only static actors can have many bodies
				const btVector3 dir = Vec3_to_btVector3(direction);
				bodies[0]->applyCentralImpulse(dir.normalized() * newtons);
			}
		}

		void BulletPhysics::VApplyTorque(const Vec3& direction, float magnitude, ActorID id)
		{
			std::vector<btRigidBody*> bodies = m_pData->GetRigidBodies(id);
			if (bodies.size() > 0)
			{
				assert(bodies.size() == 1);
				const btVector3 dir = Vec3_to_btVector3(direction);
				bodies[0]->applyTorqueImpulse(dir.normalized() * magnitude);
			}
		}

		void BulletPhysics::VStopActor(ActorID id)
		{
			VSetLinearVelocity(id, Vec3(0, 0, 0), 1.0f);
		}

		void BulletPhysics::VSetLinearVelocity(ActorID id, const Vec3& direction, float magnitude)
		{
			std::vector<btRigidBody*> bodies = m_pData->GetRigidBodies(id);
			if (bodies.size() > 0)
			{
				assert(bodies.size() == 1);
				const btVector3 dir = Vec3_to_btVector3(direction).normalized();
				bodies[0]->setLinearVelocity(dir * magnitude);
			}
		}

		void BulletPhysics::VSetGlobalGravity(Vec3& gravity)
		{
			assert(m_pData && m_pData->m_pDynamicsWorld);
			m_pData->m_pDynamicsWorld->setGravity(Vec3_to_btVector3(gravity));
		}
	}
}