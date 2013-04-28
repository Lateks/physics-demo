#include "Game.h"
#include "GameActor.h"
#include "GameData.h"
#include "IDisplay.h"
#include "TimerFactories.h"
#include "IGameLogic.h"
#include "IInputState.h"
#include "IEventManager.h"
#include "EventManager.h" // TODO: make a factory method for these.
#include "RenderingEngineFactories.h"
#include "IPhysicsEngine.h"
#include "PhysicsEngineFactories.h"
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

	Game::Game()
	{
		// Setup the display component (rendering and input handling).
		std::unique_ptr<IDisplay> pRenderer(Display::CreateRenderer());
		if (!pRenderer.get())
		{
			std::cerr << "Failed to create rendering device." << std::cerr;
			return;
		}

		if (!pRenderer->SetupAndOpenWindow(1024, 800,
			Display::DRIVER_TYPE::OPEN_GL, Display::CAMERA_TYPE::FPS))
		{
			std::cerr << "Failed to open OpenGL device." << std::cerr;
			return;
		}

		m_pData = GameData::GetInstance();
		m_pData->SetInputStateHandler(pRenderer->GetInputState());
		m_pData->SetDisplayComponent(std::shared_ptr<Display::IDisplay>(pRenderer.release()));

		std::unique_ptr<IGameLogic> pGameLogic(CreateDemoGameLogic());
		if (!pGameLogic)
		{
			std::cerr << "Failed to create demo input handler." << std::endl;
			return;
		}
		m_pData->SetInputHandler(std::shared_ptr<IGameLogic>(pGameLogic.release()));

		// Setup timer.
		std::unique_ptr<ITimer> pTimer(GetTimer());
		if (!pTimer)
		{
			std::cerr << "Failed to create a timer." << std::endl;
			return;
		}
		m_pData->setTimer(std::shared_ptr<ITimer>(pTimer.release()));

		// Setup event manager.
		std::unique_ptr<IEventManager> pEventManager(new EventManager());
		if (!pEventManager)
		{
			std::cerr << "Failed to create an event manager." << std::endl;
			return;
		}
		m_pData->SetEventManager(std::shared_ptr<Events::IEventManager>(pEventManager.release()));

		// Setup physics. World is scaled by the constant given as parameter
		// (compared to the size of the rendered world).
		std::unique_ptr<Physics::IPhysicsEngine> physics(
			Physics::CreatePhysicsEngine(0.05f));
		if (!physics)
		{
			std::cerr << "Failed to initialize physics engine." << std::endl;
			return;
		}
		m_pData->SetPhysicsEngine(std::shared_ptr<Physics::IPhysicsEngine>(physics.release()));
	}

	Game::~Game() { }

	Game::Game(Game& game)
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
		while (pDisplay->Running())
		{
			if (pDisplay->WindowActive())
			{
				gameLogic->HandleInputs();
				events->DispatchEvents();
				physics->VUpdateSimulation(frameDeltaSec);
				physics->VSyncScene();
				pDisplay->DrawScene();
			}
			else
			{
				pDisplay->YieldDevice();
			}
			timeEnd = m_pData->CurrentTimeSec();
			frameDeltaSec = timeEnd - timeBegin;
			timeBegin = timeEnd;
		}

		return 0;
	}
}