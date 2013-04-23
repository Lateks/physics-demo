#ifndef PHYSICS_ENGINE_FACTORIES_H
#define PHYSICS_ENGINE_FACTORIES_H

#include "IPhysicsEngine.h"
#include "BulletPhysics.h"
#include <memory>

namespace GameEngine
{
	namespace Physics
	{
		std::unique_ptr<IPhysicsEngine> CreatePhysicsEngine(float worldScale = 1.f)
		{
			std::unique_ptr<IPhysicsEngine> physics;
			physics.reset(new BulletPhysics(worldScale));
			if (!physics.get() || !physics->VInitEngine())
			{
				physics.reset();
				return physics;
			}
			return physics;
		}
	}
}

#endif