#include "BulletPhysics.h"
#include "BulletPhysicsData.h"
#include "GameActor.h"
#include "WorldTransformComponent.h"
#include "GameData.h"
#include "IEventManager.h"
#include "XMLPhysicsData.h"
#include "Events.h"
#include "BulletConversions.h"
#include <cassert>
#include <memory>
#include <algorithm>

using std::shared_ptr;
using std::weak_ptr;

// TODO
namespace GameEngine
{
	using LinearAlgebra::Mat4;
	using LinearAlgebra::Vec3;
	using LinearAlgebra::Quaternion;

	namespace PhysicsEngine
	{
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
			for (auto it = m_pData->m_actorToRigidBodyMap.begin();
				      it != m_pData->m_actorToRigidBodyMap.end(); it++)
			{
				ActorID id = it->first;
				const btRigidBody *body = it->second;
				const Quaternion rot = btQuaternion_to_Quaternion(body->getOrientation());
				const Vec3 pos = btVector3_to_Vec3(body->getCenterOfMassPosition());

				GameActor *pActor = game->GetActor(id);
				if (pActor)
				{
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
							event.reset(new Events::ActorMoveEvent(game->CurrentTime() / 1000.0f, id));
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

		void BulletPhysics::VAddConvexMesh(std::vector<LinearAlgebra::Vec3>& vertices,
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

		void BulletPhysics::VCreateTrigger(WeakActorPtr pActor, const float dim)
		{
			if (pActor.expired())
				return;
			StrongActorPtr pStrongActor(pActor);

			// Create a cube-shaped trigger area. Of course, this could really
			// be any convex shape. The common functionality associated with
			// adding triggers of any shape is in BulletPhysicsData::AddTriggerShape.
			btBoxShape * const boxShape =
				new btBoxShape(Vec3_to_btVector3(Vec3(dim, dim, dim)));
			m_pData->AddTriggerShape(pStrongActor, boxShape);
		}

		void BulletPhysics::VRenderDiagnostics()
		{
		}
		
		void BulletPhysics::VRemoveActor(ActorID id)
		{
		}
	}
}