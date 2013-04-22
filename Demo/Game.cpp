#include "Game.h"
#include "GameActor.h"
#include "GameData.h"
#include "IDisplay.h"
#include "TimerFactories.h"
#include "IGameInputHandler.h"
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
		std::unique_ptr<IDisplay> renderer(Display::CreateRenderer());
		if (!renderer.get())
		{
			std::cerr << "Failed to create rendering device." << std::cerr;
			return;
		}

		if (!renderer->SetupAndOpenWindow(1024, 800,
			Display::DRIVER_TYPE::OPEN_GL, Display::CAMERA_TYPE::FPS))
		{
			std::cerr << "Failed to open OpenGL device." << std::cerr;
			return;
		}

		m_pData = GameData::getInstance();
		m_pData->SetInputStateHandler(renderer->GetInputState());
		m_pData->SetRenderer(renderer.release());

		std::unique_ptr<IGameInputHandler> pInputHandler(CreateDemoInputHandler());
		if (!pInputHandler.get())
		{
			std::cerr << "Failed to create demo input handler." << std::endl;
			return;
		}
		m_pData->SetInputHandler(pInputHandler.release());

		// Setup timer.
		std::unique_ptr<ITimer> timer(GetTimer());
		if (!timer.get())
		{
			std::cerr << "Failed to create a timer." << std::endl;
			return;
		}
		m_pData->setTimer(timer.release());

		// Setup event manager.
		std::unique_ptr<IEventManager> pEventManager(new EventManager());
		if (!pEventManager.get())
		{
			std::cerr << "Failed to create an event manager." << std::endl;
			return;
		}
		m_pData->SetEventManager(pEventManager.release());

		// Setup physics.
		std::unique_ptr<Physics::IPhysicsEngine> physics(
			Physics::CreatePhysicsEngine());
		if (!physics.get())
		{
			std::cerr << "Failed to initialize physics engine." << std::endl;
			return;
		}
		m_pData->SetPhysicsEngine(physics.release());
	}

	Game::~Game()
	{
		delete m_pData;
	}

	Game::Game(Game& game)
	{
		if (this != &game)
		{
			delete m_pData;
			m_pData = game.m_pData;
			game.m_pData = nullptr;
		}
	}

	int Game::Run()
	{
		IDisplay *renderer = m_pData->GetRenderer();
		IPhysicsEngine *physics = m_pData->GetPhysicsEngine();
		IEventManager *events = m_pData->GetEventManager();
		IGameInputHandler *gameLogic = m_pData->GetInputHandler();
		if (!renderer || !physics || !events || !gameLogic)
			return 1;

		gameLogic->SetupInitialScene(m_pData);

		float timeBegin = m_pData->CurrentTimeSec();
		float timeEnd;
		float frameDeltaSec = 1.0f/60;
		while (renderer->Running())
		{
			if (renderer->WindowActive())
			{
				gameLogic->HandleInputs();
				events->DispatchEvents();
				physics->VUpdateSimulation(frameDeltaSec);
				physics->VSyncScene();
				renderer->DrawScene();
			}
			else
			{
				renderer->YieldDevice();
			}
			timeEnd = m_pData->CurrentTimeSec();
			frameDeltaSec = timeEnd - timeBegin;
			timeBegin = timeEnd;
		}

		return 0;
	}
}