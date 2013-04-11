#include "BulletPhysics.h"
#include "BulletPhysicsData.h"
#include "GameActor.h"
#include "Vec3.h"
#include "Mat4.h"
#include <cassert>

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

		void BulletPhysics::VSyncScene()
		{
			for (auto it = m_pData->m_actorToRigidBodyMap.begin();
				      it != m_pData->m_actorToRigidBodyMap.end(); it++)
			{
				ActorID id = it->first;

				// TODO: conversions needed between this and other matrix
				// types.
				const btMotionState *motionState = it->second->getMotionState();
				assert(motionState);

				// TODO: Get actor by id
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