#include "BulletPhysics.h"
#include "BulletPhysicsData.h"
#include "GameActor.h"
#include "WorldTransformComponent.h"
#include "GameData.h"
#include "IEventManager.h"
#include "Events.h"
#include "Vec3.h"
#include "Mat4.h"
#include <cassert>
#include <memory>

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

		Vec3 btVector3_to_Vec3(const btVector3& vec)
		{
			return Vec3(vec.x(), vec.y(), vec.z());
		}

		Quaternion btQuaternion_to_Quaternion(const btQuaternion& quat)
		{
			return Quaternion(quat.x(), quat.y(), quat.z(), quat.w());
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
			const Mat4& initialTransform)
		{
		}

		void BulletPhysics::VCreateTrigger(WeakActorPtr pActor,
			const Vec3& pos, const float dim)
		{
		}

		void BulletPhysics::VRenderDiagnostics()
		{
		}
		
		void BulletPhysics::VRemoveActor(ActorID id)
		{
		}
	}
}