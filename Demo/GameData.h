#ifndef GAME_DATA_H
#define GAME_DATA_H

#include "enginefwd.h"
#include <map>
#include <memory>

namespace GameEngine
{
	class GameData
	{
	private:
		static std::shared_ptr<GameData> pInstance;
		GameData() : m_pPhysicsEngine(nullptr) { }

		std::shared_ptr<Display::IDisplay> m_pDisplay;
		std::shared_ptr<Display::IInputState> m_pInputState;
		std::shared_ptr<Events::IEventManager> m_pEvents;
		std::shared_ptr<Physics::IPhysicsEngine> m_pPhysicsEngine;
		std::shared_ptr<IGameLogic> m_pInputHandler;
		std::shared_ptr<ITimer> m_pTimer;

		std::map<ActorID, std::shared_ptr<GameActor>> m_actors;
	public:
		static std::shared_ptr<GameData> GetInstance()
		{
			if (!pInstance)
			{
				pInstance.reset(new GameData());
			}
			return pInstance;
		}
		~GameData();

		float CurrentTimeSec();

		void AddActor(std::weak_ptr<GameActor> pActor);
		std::weak_ptr<GameActor> GetActor(ActorID id)
		{
			return m_actors[id];
		}
		void SetInputHandler(std::shared_ptr<IGameLogic> pInputHandler)
		{
			m_pInputHandler = pInputHandler;
		}
		std::shared_ptr<IGameLogic> GetInputHandler()
		{
			return m_pInputHandler;
		}
		void SetPhysicsEngine(std::shared_ptr<Physics::IPhysicsEngine> pPhysics)
		{
			m_pPhysicsEngine = pPhysics;
		}
		std::shared_ptr<Physics::IPhysicsEngine> GetPhysicsEngine() const
		{
			return m_pPhysicsEngine;
		}
		void SetInputStateHandler(std::shared_ptr<Display::IInputState> pInputState)
		{
			m_pInputState = std::shared_ptr<Display::IInputState>(pInputState);
		}
		std::shared_ptr<Display::IInputState> GetInputStateHandler() const
		{
			return m_pInputState;
		}
		void SetDisplayComponent(std::shared_ptr<Display::IDisplay> pDisplay)
		{
			m_pDisplay = pDisplay;
		}
		std::shared_ptr<Display::IDisplay> GetDisplayComponent() const
		{
			return m_pDisplay;
		}
		void setTimer(std::shared_ptr<ITimer> pTimer)
		{
			m_pTimer = pTimer;
		}
		std::shared_ptr<ITimer> const Timer() const
		{
			return m_pTimer;
		}
		void SetEventManager(std::shared_ptr<Events::IEventManager> pManager)
		{
			m_pEvents = pManager;
		}
		std::shared_ptr<Events::IEventManager> GetEventManager()
		{
			return m_pEvents;
		}
	};
}

#endif