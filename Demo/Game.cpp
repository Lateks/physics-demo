#include "Game.h"
#include "GameActor.h"
#include "GameData.h"
#include "IDisplay.h"
#include "ITimer.h"
#include "IGameLogic.h"
#include "IInputState.h"
#include "IEventManager.h"
#include "IPhysicsEngine.h"
#include "EngineComponentFactories.h"
#include "Vec3.h"

#include <irrlicht.h>

#include <iostream>
#include <memory>
#include <iostream>

namespace GameEngine
{
	using Display::IDisplay;
	using Physics::IPhysicsEngine;
	using Events::IEventManager;
	using Events::EventManager;

	inline void PrintError(const std::string& message)
	{
		std::cerr << message << std::endl;
	}

	Game::Game()
	{
		auto pGameData = GameData::GetInstance();

		// Setup the display component (rendering and input handling).
		std::unique_ptr<IDisplay> pRenderer(CreateRenderer());
		if (!pRenderer)
		{
			PrintError("Failed to create rendering device.");
			return;
		}

		if (!pRenderer->VSetupAndOpenWindow(1024, 800,
			Display::DRIVER_TYPE::OPEN_GL, Display::CAMERA_TYPE::FPS_WASD))
		{
			PrintError("Failed to open OpenGL device.");
			return;
		}

		pRenderer->VSetCameraFOV(75.f);
		pRenderer->VHideCursor();

		pGameData->SetInputStateHandler(pRenderer->VGetInputState());
		pGameData->SetDisplayComponent(std::shared_ptr<Display::IDisplay>(pRenderer.release()));

		// Setup timer.
		std::unique_ptr<ITimer> pTimer(CreateTimer());
		if (!pTimer)
		{
			PrintError("Failed to create a timer.");
			return;
		}
		pGameData->SetTimer(std::shared_ptr<ITimer>(pTimer.release()));

		// Setup the main game logic handler (handles inputs etc.).
		std::unique_ptr<IGameLogic> pGameLogic(CreateDemoGameLogic());
		if (!pGameLogic)
		{
			PrintError("Failed to create demo input handler.");
			return;
		}
		pGameData->SetInputHandler(std::shared_ptr<IGameLogic>(pGameLogic.release()));

		// Setup event manager.
		std::unique_ptr<IEventManager> pEventManager(CreateEventManager());
		if (!pEventManager)
		{
			PrintError("Failed to create an event manager.");
			return;
		}
		pGameData->SetEventManager(std::shared_ptr<Events::IEventManager>(pEventManager.release()));

		// Setup physics. World is scaled by the constant given as parameter
		// (compared to the size of the rendered world).
		std::unique_ptr<Physics::IPhysicsEngine> physics(
			CreatePhysicsEngine(0.05f));
		if (!physics)
		{
			PrintError("Failed to initialize physics engine.");
			return;
		}
		pGameData->SetPhysicsEngine(std::shared_ptr<Physics::IPhysicsEngine>(physics.release()));
	}

	Game::~Game() { }

	bool Game::Run()
	{
		auto pGameData = GameData::GetInstance();
		auto pDisplay = pGameData->GetDisplayComponent();
		auto pPhysics = pGameData->GetPhysicsEngine();
		auto pEvents = pGameData->GetEventManager();
		auto pGameLogic = pGameData->GetInputHandler();
		if (!pDisplay || !pPhysics || !pEvents || !pGameLogic)
			return false;

		pGameLogic->VSetupInitialScene();

		float timeBegin = pGameData->CurrentTimeSec();
		float timeEnd;
		float frameDeltaSec = 1.0f/60;
		while (pDisplay->VRunning())
		{
			if (pDisplay->VWindowActive())
			{
				pGameLogic->VUpdate();
				pEvents->VDispatchEvents();
				pPhysics->VUpdateSimulation(frameDeltaSec);
				pPhysics->VSyncScene();
				pDisplay->VDrawScene();
			}
			else
			{
				pDisplay->VYieldDevice();
			}
			timeEnd = pGameData->CurrentTimeSec();
			frameDeltaSec = timeEnd - timeBegin;
			timeBegin = timeEnd;
		}

		return true;
	}
}