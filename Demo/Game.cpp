#include "Game.h"
#include "GameActor.h"
#include "GameData.h"
#include "IDisplay.h"
#include "ITimer.h"
#include "IGameLogic.h"
#include "IInputState.h"
#include "IPhysicsEngine.h"
#include "EventManager.h"

#include <iostream>
#include <memory>

namespace GameEngine
{
	inline void PrintError(const std::string& message)
	{
		std::cerr << message << std::endl;
	}

	enum class EngineState
	{
		UNINITIALIZED,
		INITIALIZATION_FAILED,
		INITIALIZED
	};

	struct GameImpl
	{
		GameImpl() : m_state(EngineState::UNINITIALIZED) { }
		EngineState m_state;

		inline void Fail() { m_state = EngineState::INITIALIZATION_FAILED; }
		inline void Success() { m_state = EngineState::INITIALIZED; }
		bool CheckInitOkAndPrintErrors()
		{
			if (m_state == EngineState::UNINITIALIZED)
			{
				PrintError("Game: Cannot run, engine is uninitialized. Initialize before calling Run.");
				return false;
			}
			else if (m_state == EngineState::INITIALIZATION_FAILED)
			{
				PrintError("Game: Cannot run, initialization of some engine components failed.");
				return false;
			}
			return true;
		}
		void Shutdown();
	};

	// Sets up the GameData singleton.
	bool Game::Initialize(const IGameLogicFactory& gameLogicFactory,
		const Display::IDisplayFactory& displayFactory,
		const Physics::IPhysicsEngineFactory& physicsFactory)
	{
		auto pGameData = GameData::GetInstance();

		// Setup the display component (rendering and input handling).
		auto pDisplay = displayFactory.VCreateDeviceAndOpenWindow();
		if (!pDisplay)
		{
			PrintError("Failed to initialize rendering device.");
			m_pImpl->Fail();
			return false;
		}

		pGameData->SetDisplayComponent(pDisplay);
		pGameData->SetInputStateHandler(pDisplay->VGetInputState());

		// Setup event manager.
		auto pEventManager = std::make_shared<Events::EventManager>();
		if (!pEventManager)
		{
			PrintError("Failed to initialize an event manager.");
			m_pImpl->Fail();
			return false;
		}
		pGameData->SetEventManager(pEventManager);

		// Setup the physics engine.
		auto pPhysics = physicsFactory.CreatePhysicsEngine();
		if (!pPhysics)
		{
			PrintError("Failed to initialize physics engine.");
			m_pImpl->Fail();
			return false;
		}
		pGameData->SetPhysicsEngine(pPhysics);

		// Setup the main game logic handler (handles inputs etc.).
		// The factory also sets up the initial scene.
		auto pGameLogic = gameLogicFactory.CreateGameLogic();
		if (!pGameLogic)
		{
			PrintError("Failed to initialize game logic instance.");
			m_pImpl->Fail();
			return false;
		}
		pGameData->SetInputHandler(pGameLogic);

		m_pImpl->Success();
		return true;
	}

	void GameImpl::Shutdown()
	{
		auto pGameData = GameData::GetInstance();
		pGameData->SetInputHandler(nullptr);
		pGameData->SetPhysicsEngine(nullptr);
		pGameData->SetEventManager(nullptr);
		pGameData->SetDisplayComponent(nullptr);
	}

	Game::Game() : m_pImpl(new GameImpl()) { }

	Game::~Game() { }

	bool Game::Run() const
	{
		if (!m_pImpl->CheckInitOkAndPrintErrors())
			return false;

		auto pGameData = GameData::GetInstance();
		auto pDisplay = pGameData->GetDisplayComponent();
		auto pPhysics = pGameData->GetPhysicsEngine();
		auto pEvents = pGameData->GetEventManager();
		auto pGameLogic = pGameData->GetInputHandler();

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

		m_pImpl->Shutdown();
		return true;
	}
}