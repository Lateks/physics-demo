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
	inline void PrintError(const std::string& message)
	{
		std::cerr << message << std::endl;
	}

	// Sets up the GameData singleton.
	bool Game::Initialize()
	{
		auto pGameData = GameData::GetInstance();

		// Setup the display component (rendering and input handling).
		auto pDisplay = Demo::CreateRenderer(1024, 800,
			Display::DriverType::OPEN_GL, Display::CameraType::FPS_WASD);
		if (!pDisplay)
		{
			PrintError("Failed to create rendering device.");
			return false;
		}

		pGameData->SetDisplayComponent(pDisplay);
		pGameData->SetInputStateHandler(pDisplay->VGetInputState());

		// Setup the main game logic handler (handles inputs etc.).
		auto pGameLogic = Demo::CreateDemoGameLogic();
		if (!pGameLogic)
		{
			PrintError("Failed to create demo input handler.");
			return false;
		}
		pGameData->SetInputHandler(pGameLogic);

		// Setup event manager.
		auto pEventManager = Demo::CreateEventManager();
		if (!pEventManager)
		{
			PrintError("Failed to create an event manager.");
			return false;
		}
		pGameData->SetEventManager(pEventManager);

		// Setup physics. World is scaled by the constant given as parameter
		// (compared to the size of the rendered world).
		auto pPhysics = Demo::CreatePhysicsEngine(0.05f);
		if (!pPhysics || !pPhysics->VInitEngine("..\\assets\\materials.xml"))
		{
			PrintError("Failed to initialize physics engine.");
			return false;
		}
		pGameData->SetPhysicsEngine(pPhysics);
		return true;
	}

	void Game::Shutdown()
	{
		auto pGameData = GameData::GetInstance();
		pGameData->SetPhysicsEngine(nullptr);
		pGameData->SetEventManager(nullptr);
		pGameData->SetInputHandler(nullptr);
		pGameData->SetDisplayComponent(nullptr);
	}

	Game::Game() { }

	Game::~Game() { }

	bool Game::Run()
	{
		if (!Initialize())
		{
			PrintError("Initialization of some engine components failed.");
			return false;
		}

		auto pGameData = GameData::GetInstance();
		auto pDisplay = pGameData->GetDisplayComponent();
		auto pPhysics = pGameData->GetPhysicsEngine();
		auto pEvents = pGameData->GetEventManager();
		auto pGameLogic = pGameData->GetInputHandler();

		if (!pGameLogic->VSetupInitialScene())
		{
			PrintError("Failed to setup the initial scene.");
			return false;
		}

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

		Shutdown();
		return true;
	}
}