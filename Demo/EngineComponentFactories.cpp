#include "EngineComponentFactories.h"
#include "BulletPhysics.h"
#include "IrrlichtDisplay.h"
#include "GameData.h"
#include "EventManager.h"
#include "DemoGameLogic.h"

using namespace GameEngine;

namespace Demo
{
	std::shared_ptr<Display::IDisplay> CreateRenderer(int width, int height,
		Display::DriverType driverType, Display::CameraType cameraType)
	{
		Display::IrrlichtDisplayFactory factory(width, height, driverType, cameraType);
		return factory.VCreateDeviceAndOpenWindow();
	}

	std::shared_ptr<Events::IEventManager> CreateEventManager()
	{
		return std::make_shared<Events::EventManager>();
	}

	std::shared_ptr<Physics::IPhysicsEngine> CreatePhysicsEngine(const std::string& materialFileName, float worldScale)
	{
		Physics::BulletPhysicsFactory factory(materialFileName, worldScale);
		return factory.CreatePhysicsEngine();
	}

	std::shared_ptr<GameEngine::IGameLogic> CreateDemoGameLogic()
	{
		DemoGameLogicFactory factory;
		return factory.CreateGameLogic();
	}
}