#include "EngineComponentFactories.h"
#include "BulletPhysics.h"
#include "IrrlichtDisplay.h"
#include "GameData.h"
#include "EventManager.h"

namespace GameEngine
{
	std::unique_ptr<Display::IDisplay> CreateRenderer()
	{
		return std::unique_ptr<Display::IDisplay>(new Display::IrrlichtDisplay());
	}

	std::unique_ptr<Events::IEventManager> CreateEventManager()
	{
		return std::unique_ptr<Events::IEventManager>(new Events::EventManager());
	}

	std::unique_ptr<Physics::IPhysicsEngine> CreatePhysicsEngine(float worldScale)
	{
		std::unique_ptr<Physics::IPhysicsEngine> physics(new Physics::BulletPhysics(worldScale));
		return physics;
	}

	std::unique_ptr<ITimer> CreateTimer()
	{
		std::unique_ptr<ITimer> timer;
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
}