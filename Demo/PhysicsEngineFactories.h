#ifndef PHYSICS_ENGINE_FACTORIES_H
#define PHYSICS_ENGINE_FACTORIES_H

#include "IPhysicsEngine.h"
#include "BulletPhysics.h"

namespace GameEngine
{
	namespace PhysicsEngine
	{
		IPhysicsEngine *CreatePhysicsEngine()
		{
			IPhysicsEngine *physics = new BulletPhysics();
			if (!physics->VInitEngine())
			{
				return nullptr;
			}
			return physics;
		}
	}
}

#endif