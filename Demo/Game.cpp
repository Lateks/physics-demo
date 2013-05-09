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

using namespace GameEngine;

namespace Demo
{
	inline void PrintError(const std::string& message)
	{
		std::cerr << message << std::endl;
	}

	// Sets up the GameData singleton.
	bool Setup()
	{
		auto pGameData = GameData::GetInstance();

		// Setup the display component (rendering and input handling).
		auto pRenderer = CreateRenderer();
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
		pGameData->SetDisplayComponent(pRenderer);

		// Setup timer.
		auto pTimer = CreateTimer();
		if (!pTimer)
		{
			PrintError("Failed to create a timer.");
			return false;
		}
		pGameData->SetTimer(pTimer);

		// Setup the main game logic handler (handles inputs etc.).
		auto pGameLogic = CreateDemoGameLogic();
		if (!pGameLogic)
		{
			PrintError("Failed to create demo input handler.");
			return false;
		}
		pGameData->SetInputHandler(pGameLogic);

		// Setup event manager.
		auto pEventManager = CreateEventManager();
		if (!pEventManager)
		{
			PrintError("Failed to create an event manager.");
			return false;
		}
		pGameData->SetEventManager(pEventManager);

		// Setup physics. World is scaled by the constant given as parameter
		// (compared to the size of the rendered world).
		auto pPhysics = CreatePhysicsEngine(0.05f);
		if (!pPhysics || !pPhysics->VInitEngine("..\\assets\\materials.xml"))
		{
			PrintError("Failed to initialize physics engine.");
			return false;
		}
		pGameData->SetPhysicsEngine(pPhysics);
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