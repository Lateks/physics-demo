#include "BulletPhysics.h"
#include "BulletPhysicsData.h"
#include "GameActor.h"
#include "WorldTransformComponent.h"
#include "GameData.h"
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

				const WorldTransformConversion * const motionState =
					static_cast<const WorldTransformConversion * const>(
					it->second->getMotionState());
				assert(motionState);

				GameActor *pActor = game->GetActor(id);
				if (pActor && motionState)
				{
					weak_ptr<WorldTransformComponent> pWeakWorldTrans = pActor->GetWorldTransform();
					if (!pWeakWorldTrans.expired())
					{
						shared_ptr<WorldTransformComponent> pWorldTrans(pWeakWorldTrans);
						if (pWorldTrans->GetTransform() != motionState->m_transform)
						{
							pWorldTrans->SetTransform(motionState->m_transform);
							// TODO: send an event about this movement!
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