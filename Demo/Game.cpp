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

#include <iostream>
#include <memory>

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

	// Sets up the GameData singleton.
	bool Setup()
	{
		auto pGameData = GameData::GetInstance();

		// Setup the display component (rendering and input handling).
		std::unique_ptr<IDisplay> pRenderer(CreateRenderer());
		if (!pRenderer)
		{
			PrintError("Failed to create rendering device.");
			return false;
		}

		if (!pRenderer->VSetupAndOpenWindow(1024, 800,
			Display::DRIVER_TYPE::OPEN_GL, Display::CAMERA_TYPE::FPS_WASD))
		{
			PrintError("Failed to open OpenGL device.");
			return false;
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
			return false;
		}
		pGameData->SetTimer(std::shared_ptr<ITimer>(pTimer.release()));

		// Setup the main game logic handler (handles inputs etc.).
		std::unique_ptr<IGameLogic> pGameLogic(CreateDemoGameLogic());
		if (!pGameLogic)
		{
			PrintError("Failed to create demo input handler.");
			return false;
		}
		pGameData->SetInputHandler(std::shared_ptr<IGameLogic>(pGameLogic.release()));

		// Setup event manager.
		std::unique_ptr<IEventManager> pEventManager(CreateEventManager());
		if (!pEventManager)
		{
			PrintError("Failed to create an event manager.");
			return false;
		}
		pGameData->SetEventManager(std::shared_ptr<Events::IEventManager>(pEventManager.release()));

		// Setup physics. World is scaled by the constant given as parameter
		// (compared to the size of the rendered world).
		std::unique_ptr<Physics::IPhysicsEngine> physics(
			CreatePhysicsEngine(0.05f));
		if (!physics || !physics->VInitEngine("..\\assets\\materials.xml"))
		{
			PrintError("Failed to initialize physics engine.");
			return false;
		}
		pGameData->SetPhysicsEngine(std::shared_ptr<Physics::IPhysicsEngine>(physics.release()));
		return true;
	}

	Game::Game() { }

	Game::~Game() { }

	bool Game::Run()
	{
		if (!Setup())
			return false;

		auto pGameData = GameData::GetInstance();
		auto pDisplay = pGameData->GetDisplayComponent();
		auto pPhysics = pGameData->GetPhysicsEngine();
		auto pEvents = pGameData->GetEventManager();
		auto pGameLogic = pGameData->GetInputHandler();

		if (!pGameLogic->VSetupInitialScene())
			return false;

		unsigned int timeBegin = pGameData->CurrentTimeMs();
		unsigned int timeEnd;
		float frameDeltaSec = 1.0f/60;
		while (pDisplay->VRunning())
		{
			if (pDisplay->VWindowActive())
			{
				pGameLogic->VUpdate(frameDeltaSec);
				pEvents->VDispatchEvents();
				pPhysics->VUpdateSimulation(frameDeltaSec);
				pPhysics->VSyncScene();
				pDisplay->VDrawScene();
			}
			else
			{
				pDisplay->VYieldDevice();
			}
			timeEnd = pGameData->CurrentTimeMs();
			frameDeltaSec = (timeEnd - timeBegin) / 1000.f;
			timeBegin = timeEnd;
		}

		return true;
	}
}