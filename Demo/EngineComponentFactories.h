#pragma once

#include "enginefwd.h"
#include "IDisplay.h"

namespace Demo
{
	std::shared_ptr<GameEngine::Display::IDisplay> CreateRenderer(int width, int height,
		GameEngine::Display::DriverType driverType, GameEngine::Display::CameraType cameraType);
	std::shared_ptr<GameEngine::Physics::IPhysicsEngine> CreatePhysicsEngine(float worldScale = 1.f);
	std::shared_ptr<GameEngine::ITimer> CreateTimer();
	std::shared_ptr<GameEngine::Events::IEventManager> CreateEventManager();
	std::shared_ptr<GameEngine::IGameLogic> CreateDemoGameLogic();
}