#include "GameData.h"
#include "GameActor.h"
#include "IDisplay.h"
#include "IPhysicsEngine.h"
#include "IEventManager.h"
#include "ITimer.h"
#include <algorithm>
#include <map>

namespace GameEngine
{
	std::shared_ptr<GameData> GameData::pInstance;

	GameData::~GameData() { }

	float GameData::CurrentTimeSec()
	{
		return m_pTimer->GetTimeMs() / 1000.0f;
	}

	void GameData::AddActor(WeakActorPtr pActor)
	{
		if (pActor.expired())
			return;

		StrongActorPtr pStrongActor(pActor);
		m_actors[pStrongActor->GetID()] = pStrongActor; 
	}

	std::shared_ptr<GameData> GameData::GetInstance()
	{
		if (!pInstance)
		{
			pInstance.reset(new GameData());
		}
		return pInstance;
	}

	std::weak_ptr<GameActor> GameData::GetActor(ActorID id)
	{
		return m_actors[id];
	}

	void GameData::SetInputHandler(std::shared_ptr<IGameLogic> pInputHandler)
	{
		m_pInputHandler = pInputHandler;
	}

	std::shared_ptr<IGameLogic> GameData::GetInputHandler() const
	{
		return m_pInputHandler;
	}

	void GameData::SetPhysicsEngine(std::shared_ptr<Physics::IPhysicsEngine> pPhysics)
	{
		m_pPhysicsEngine = pPhysics;
	}

	std::shared_ptr<Physics::IPhysicsEngine> GameData::GetPhysicsEngine() const
	{
		return m_pPhysicsEngine;
	}

	void GameData::SetInputStateHandler(std::shared_ptr<Display::IInputState> pInputState)
	{
		m_pInputState = std::shared_ptr<Display::IInputState>(pInputState);
	}

	std::shared_ptr<Display::IInputState> GameData::GetInputStateHandler() const
	{
		return m_pInputState;
	}

	void GameData::SetDisplayComponent(std::shared_ptr<Display::IDisplay> pDisplay)
	{
		m_pDisplay = pDisplay;
	}

	std::shared_ptr<Display::IDisplay> GameData::GetDisplayComponent() const
	{
		return m_pDisplay;
	}

	void GameData::setTimer(std::shared_ptr<ITimer> pTimer)
	{
		m_pTimer = pTimer;
	}

	std::shared_ptr<ITimer> GameData::Timer() const
	{
		return m_pTimer;
	}

	void GameData::SetEventManager(std::shared_ptr<Events::IEventManager> pManager)
	{
		m_pEvents = pManager;
	}

	std::shared_ptr<Events::IEventManager> GameData::GetEventManager() const
	{
		return m_pEvents;
	}
}