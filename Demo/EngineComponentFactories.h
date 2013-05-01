#ifndef ENGINE_COMPONENT_FACTORIES_H
#define ENGINE_COMPONENT_FACTORIES_H

#include "enginefwd.h"

namespace GameEngine
{
	std::unique_ptr<Display::IDisplay> CreateRenderer();
	std::unique_ptr<Physics::IPhysicsEngine> CreatePhysicsEngine(float worldScale = 1.f);
	std::unique_ptr<ITimer> CreateTimer();
}

#endif