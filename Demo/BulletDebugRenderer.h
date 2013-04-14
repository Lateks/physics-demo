#ifndef BULLET_DEBUG_RENDERER_H
#define BULLET_DEBUG_RENDERER_H

#include <btBulletCollisionCommon.h>

namespace GameEngine
{
	namespace PhysicsEngine
	{
		// TODO: not implemented yet
		class BulletDebugRenderer : public btIDebugDraw
		{
		public:
			virtual void drawLine(const btVector3& from,
								  const btVector3& to,
								  const btVector3& color);
			virtual void drawContactPoint(const btVector3& PointOnB,
										  const btVector3& normalOnB,
										  btScalar distance,
										  int lifeTime,
										  const btVector3& color);
			virtual void reportErrorWarning(const char *warningString);
			virtual void draw3dText(const btVector3& location,
									const char *textString);
			virtual void setDebugMode(int debugMode);
			virtual int getDebugMode() const;
		};
	}
}

#endif