#include "EngineComponentFactories.h"
#include "BulletPhysics.h"
#include "IrrlichtDisplay.h"
#include "GameData.h"
#include "EventManager.h"
#include "DemoGameLogic.h"

using namespace GameEngine;

namespace Demo
{
	std::shared_ptr<Display::IDisplay> CreateRenderer()
	{
		return std::make_shared<Display::IrrlichtDisplay>();
	}

	std::shared_ptr<Events::IEventManager> CreateEventManager()
	{
		return std::make_shared<Events::EventManager>();
	}

	std::shared_ptr<Physics::IPhysicsEngine> CreatePhysicsEngine(float worldScale)
	{
		return std::make_shared<Physics::BulletPhysics>(worldScale);
	}

	std::shared_ptr<ITimer> CreateTimer()
	{
		std::shared_ptr<ITimer> timer;
		auto pDisplay = GameEngine::GameData::GetInstance()->GetDisplayComponent();

		if (pDisplay)
		{
			std::shared_ptr<Display::IrrlichtDisplay> pIrrlicht =
				std::dynamic_pointer_cast<Display::IrrlichtDisplay>(pDisplay);
			if (pIrrlicht)
			{
				timer.reset(new Display::IrrlichtTimer(pIrrlicht));
			}
		}
		return timer;
	}

	std::shared_ptr<GameEngine::IGameLogic> CreateDemoGameLogic()
	{
		return std::make_shared<Demo::DemoGameLogic>();
	}
}