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

		m_pData = GameData::GetInstance();
		m_pData->SetInputStateHandler(pRenderer->VGetInputState());
		m_pData->SetDisplayComponent(std::shared_ptr<Display::IDisplay>(pRenderer.release()));

		std::unique_ptr<IGameLogic> pGameLogic(CreateDemoGameLogic());
		if (!pGameLogic)
		{
			PrintError("Failed to create demo input handler.");
			return;
		}
		m_pData->SetInputHandler(std::shared_ptr<IGameLogic>(pGameLogic.release()));

		// Setup timer.
		std::unique_ptr<ITimer> pTimer(CreateTimer());
		if (!pTimer)
		{
			PrintError("Failed to create a timer.");
			return;
		}
		m_pData->SetTimer(std::shared_ptr<ITimer>(pTimer.release()));

		// Setup event manager.
		std::unique_ptr<IEventManager> pEventManager(CreateEventManager());
		if (!pEventManager)
		{
			PrintError("Failed to create an event manager.");
			return;
		}
		m_pData->SetEventManager(std::shared_ptr<Events::IEventManager>(pEventManager.release()));

		// Setup physics. World is scaled by the constant given as parameter
		// (compared to the size of the rendered world).
		std::unique_ptr<Physics::IPhysicsEngine> physics(
			CreatePhysicsEngine(0.05f));
		if (!physics)
		{
			PrintError("Failed to initialize physics engine.");
			return;
		}
		m_pData->SetPhysicsEngine(std::shared_ptr<Physics::IPhysicsEngine>(physics.release()));
	}

	Game::~Game() { }

	Game::Game(Game&& game)
	{
		if (this != &game)
		{
			m_pData = game.m_pData;
			game.m_pData = nullptr;
		}
	}

	int Game::Run()
	{
		auto pDisplay = m_pData->GetDisplayComponent();
		auto physics = m_pData->GetPhysicsEngine();
		auto events = m_pData->GetEventManager();
		auto gameLogic = m_pData->GetInputHandler();
		if (!pDisplay || !physics || !events || !gameLogic)
			return 1;

		gameLogic->SetupInitialScene();

		float timeBegin = m_pData->CurrentTimeSec();
		float timeEnd;
		float frameDeltaSec = 1.0f/60;
		while (pDisplay->VRunning())
		{
			if (pDisplay->VWindowActive())
			{
				gameLogic->HandleInputs();
				events->DispatchEvents();
				physics->VUpdateSimulation(frameDeltaSec);
				physics->VSyncScene();
				pDisplay->VDrawScene();
			}
			else
			{
				pDisplay->VYieldDevice();
			}
			timeEnd = m_pData->CurrentTimeSec();
			frameDeltaSec = timeEnd - timeBegin;
			timeBegin = timeEnd;
		}

		return 0;
	}
}