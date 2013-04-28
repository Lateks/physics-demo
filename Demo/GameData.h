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
		GameData() { }

		std::shared_ptr<Display::IDisplay> m_pDisplay;
		std::shared_ptr<Display::IInputState> m_pInputState;
		std::shared_ptr<Events::IEventManager> m_pEvents;
		std::shared_ptr<Physics::IPhysicsEngine> m_pPhysicsEngine;
		std::shared_ptr<IGameLogic> m_pInputHandler;
		std::shared_ptr<ITimer> m_pTimer;

		std::map<ActorID, std::shared_ptr<GameActor>> m_actors;
	public:
		static std::shared_ptr<GameData> GetInstance();
		~GameData();

		float CurrentTimeSec();

		void AddActor(std::weak_ptr<GameActor> pActor);
		std::weak_ptr<GameActor> GetActor(ActorID id);

		void SetInputHandler(std::shared_ptr<IGameLogic> pInputHandler);
		std::shared_ptr<IGameLogic> GetInputHandler() const;

		void SetPhysicsEngine(std::shared_ptr<Physics::IPhysicsEngine> pPhysics);
		std::shared_ptr<Physics::IPhysicsEngine> GetPhysicsEngine() const;

		void SetInputStateHandler(std::shared_ptr<Display::IInputState> pInputState);
		std::shared_ptr<Display::IInputState> GetInputStateHandler() const;

		void SetDisplayComponent(std::shared_ptr<Display::IDisplay> pDisplay);
		std::shared_ptr<Display::IDisplay> GetDisplayComponent() const;

		void setTimer(std::shared_ptr<ITimer> pTimer);
		std::shared_ptr<ITimer> Timer() const;

		void SetEventManager(std::shared_ptr<Events::IEventManager> pManager);
		std::shared_ptr<Events::IEventManager> GetEventManager() const;
	};
}

#endif