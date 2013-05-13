#include "GameData.h"
#include "GameActor.h"
#include "IDisplay.h"
#include "IPhysicsEngine.h"
#include "IEventManager.h"
#include "ITimer.h"
#include <algorithm>
#include <map>

namespace
{
	std::shared_ptr<GameEngine::GameData> pInstance;
}

namespace GameEngine
{
	struct GameDataImpl
	{
		std::shared_ptr<Display::IDisplay> m_pDisplay;
		std::shared_ptr<Display::IInputState> m_pInputState;
		std::shared_ptr<Events::IEventManager> m_pEvents;
		std::shared_ptr<Physics::IPhysicsEngine> m_pPhysicsEngine;
		std::shared_ptr<IGameLogic> m_pInputHandler;
		std::shared_ptr<ITimer> m_pTimer;

		std::map<ActorID, std::shared_ptr<GameActor>> m_actors;
	};

	GameData::GameData() : m_pData(new GameDataImpl()) { }

	GameData::~GameData() { }

	float GameData::CurrentTimeSec()
	{
		return CurrentTimeMs() / 1000.0f;
	}

	unsigned int GameData::CurrentTimeMs()
	{
		if (m_pData->m_pTimer)
		{
			return m_pData->m_pTimer->GetTimeMs();
		}
		return 0;
	}

	void GameData::AddActor(ActorPtr pActor)
	{
		if (!pActor)
			return;

		m_pData->m_actors[pActor->GetID()] = pActor; 
	}

	void GameData::RemoveActor(ActorID id)
	{
		auto iter = m_pData->m_actors.find(id);
		if (iter != m_pData->m_actors.end())
		{
			m_pData->m_actors.erase(iter);
		}
	}

	std::shared_ptr<GameData> GameData::GetInstance()
	{
		if (!pInstance)
		{
			pInstance.reset(new GameData());
		}
		return pInstance;
	}

	std::shared_ptr<GameActor> GameData::GetActor(ActorID id)
	{
		return m_pData->m_actors[id];
	}

	void GameData::SetInputHandler(std::shared_ptr<IGameLogic> pInputHandler)
	{
		m_pData->m_pInputHandler.reset();
		m_pData->m_pInputHandler = pInputHandler;
	}

	std::shared_ptr<IGameLogic> GameData::GetInputHandler() const
	{
		return m_pData->m_pInputHandler;
	}

	void GameData::SetPhysicsEngine(std::shared_ptr<Physics::IPhysicsEngine> pPhysics)
	{
		m_pData->m_pPhysicsEngine.reset();
		m_pData->m_pPhysicsEngine = pPhysics;
	}

	std::shared_ptr<Physics::IPhysicsEngine> GameData::GetPhysicsEngine() const
	{
		return m_pData->m_pPhysicsEngine;
	}

	void GameData::SetInputStateHandler(std::shared_ptr<Display::IInputState> pInputState)
	{
		m_pData->m_pInputState.reset();
		m_pData->m_pInputState = std::shared_ptr<Display::IInputState>(pInputState);
	}

	std::shared_ptr<Display::IInputState> GameData::GetInputStateHandler() const
	{
		return m_pData->m_pInputState;
	}

	void GameData::SetDisplayComponent(std::shared_ptr<Display::IDisplay> pDisplay)
	{
		m_pData->m_pDisplay.reset();
		m_pData->m_pDisplay = pDisplay;
	}

	std::shared_ptr<Display::IDisplay> GameData::GetDisplayComponent() const
	{
		return m_pData->m_pDisplay;
	}

	void GameData::SetTimer(std::shared_ptr<ITimer> pTimer)
	{
		m_pData->m_pTimer.reset();
		m_pData->m_pTimer = pTimer;
	}

	std::shared_ptr<ITimer> GameData::GetTimer() const
	{
		return m_pData->m_pTimer;
	}

	void GameData::SetEventManager(std::shared_ptr<Events::IEventManager> pManager)
	{
		m_pData->m_pEvents.reset();
		m_pData->m_pEvents = pManager;
	}

	std::shared_ptr<Events::IEventManager> GameData::GetEventManager() const
	{
		return m_pData->m_pEvents;
	}
}