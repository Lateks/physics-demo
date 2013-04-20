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
		static GameData *instance;
		GameData() : m_pRenderer(nullptr), m_pPhysicsEngine(nullptr) { }
		Display::IDisplay *m_pRenderer;
		ITimer *m_pTimer;
		Events::IEventManager *m_pEvents;
		Physics::IPhysicsEngine *m_pPhysicsEngine;
		std::shared_ptr<Display::IInputState> m_pInputState;
		std::map<ActorID, std::shared_ptr<GameActor>> m_actors;
	public:
		static GameData *getInstance()
		{
			if (!instance)
			{
				instance = new GameData();
			}
			return instance;
		}
		~GameData();

		float CurrentTimeSec();

		void AddActor(std::weak_ptr<GameActor> pActor);
		std::weak_ptr<GameActor> GetActor(ActorID id)
		{
			return m_actors[id];
		}
		void SetPhysicsEngine(Physics::IPhysicsEngine *physics)
		{
			m_pPhysicsEngine = physics;
		}
		Physics::IPhysicsEngine * const GetPhysicsEngine() const
		{
			return m_pPhysicsEngine;
		}
		void SetInputStateHandler(std::weak_ptr<Display::IInputState> inputState)
		{
			if (!inputState.expired())
			{
				m_pInputState = std::shared_ptr<Display::IInputState>(inputState);
			}
		}
		std::shared_ptr<Display::IInputState> GetInputStateHandler() const
		{
			return m_pInputState;
		}
		void SetRenderer(Display::IDisplay *renderer)
		{
			m_pRenderer = renderer;
		}
		Display::IDisplay * const GetRenderer() const
		{
			return m_pRenderer;
		}
		void setTimer(ITimer *timer)
		{
			m_pTimer = timer;
		}
		const ITimer * const Timer() const
		{
			return m_pTimer;
		}
		void SetEventManager(Events::IEventManager *manager)
		{
			m_pEvents = manager;
		}
		Events::IEventManager *GetEventManager()
		{
			return m_pEvents;
		}
	};
}

#endif