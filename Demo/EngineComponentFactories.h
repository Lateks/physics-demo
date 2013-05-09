#pragma once

#include "enginefwd.h"

namespace Demo
{
	std::shared_ptr<GameEngine::Display::IDisplay> CreateRenderer();
	std::shared_ptr<GameEngine::Physics::IPhysicsEngine> CreatePhysicsEngine(float worldScale = 1.f);
	std::shared_ptr<GameEngine::ITimer> CreateTimer();
	std::shared_ptr<GameEngine::Events::IEventManager> CreateEventManager();
	std::shared_ptr<GameEngine::IGameLogic> CreateDemoGameLogic();
}