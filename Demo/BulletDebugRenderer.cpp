#include "BulletDebugRenderer.h"
#include "IrrlichtRenderer.h"

namespace GameEngine
{
	namespace PhysicsEngine
	{
		void BulletDebugRenderer::drawLine(const btVector3& from,
			const btVector3& to, const btVector3& color)
		{
		}

		void BulletDebugRenderer::drawContactPoint(const btVector3& PointOnB,
			const btVector3& normalOnB, btScalar distance,
			int lifeTime, const btVector3& color)
		{
		}

		void BulletDebugRenderer::reportErrorWarning(const char *warningString)
		{
		}

		void BulletDebugRenderer::draw3dText(const btVector3& location,
			const char *textString)
		{
		}

		void BulletDebugRenderer::setDebugMode(int debugMode)
		{
		}

		int BulletDebugRenderer::getDebugMode() const
		{
			return 0;
		}
	}
}