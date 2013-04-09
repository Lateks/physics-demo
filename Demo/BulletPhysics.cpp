#include "BulletPhysics.h"
#include "BulletPhysicsData.h"
#include "GameActor.h"
#include "Vec3.h"
#include "Mat4.h"

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

		bool BulletPhysics::VSyncScene()
		{
			return false;
		}

		bool BulletPhysics::VUpdateSimulation(float deltaSec)
		{
			return false;
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