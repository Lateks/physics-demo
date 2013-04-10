#ifndef PHYSICS_ENGINE_FACTORIES_H
#define PHYSICS_ENGINE_FACTORIES_H

#include "IPhysicsEngine.h"
#include "BulletPhysics.h"
#include <memory>

namespace GameEngine
{
	namespace PhysicsEngine
	{
		std::auto_ptr<IPhysicsEngine> CreatePhysicsEngine()
		{
			std::auto_ptr<IPhysicsEngine> physics;
			physics.reset(new BulletPhysics());
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