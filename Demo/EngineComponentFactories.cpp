#include "EngineComponentFactories.h"
#include "BulletPhysics.h"
#include "IrrlichtDisplay.h"
#include "GameData.h"

namespace GameEngine
{
	std::unique_ptr<Display::IDisplay> CreateRenderer()
	{
		return std::unique_ptr<Display::IDisplay>(new Display::IrrlichtDisplay());
	}

	std::unique_ptr<Physics::IPhysicsEngine> CreatePhysicsEngine(float worldScale)
	{
		std::unique_ptr<Physics::IPhysicsEngine> physics(new Physics::BulletPhysics(worldScale));
		if (!physics || !physics->VInitEngine())
		{
			physics.reset();
		}
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