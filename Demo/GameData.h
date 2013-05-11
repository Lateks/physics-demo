#pragma once

#include "enginefwd.h"
#include <memory>

namespace GameEngine
{
	struct GameDataImpl;

	class GameData
	{
	public:
		friend class Game;

		static std::shared_ptr<GameData> GetInstance();
		~GameData();

		float CurrentTimeSec();
		unsigned int CurrentTimeMs();

		void AddActor(std::shared_ptr<GameActor> pActor);
		void RemoveActor(ActorID id);
		std::shared_ptr<GameActor> GetActor(ActorID id);

		std::shared_ptr<IGameLogic> GetInputHandler() const;
		std::shared_ptr<Physics::IPhysicsEngine> GetPhysicsEngine() const;
		std::shared_ptr<Display::IInputState> GetInputStateHandler() const;
		std::shared_ptr<Display::IDisplay> GetDisplayComponent() const;
		std::shared_ptr<ITimer> GetTimer() const;
		std::shared_ptr<Events::IEventManager> GetEventManager() const;
	private:
		GameData();
		GameData(const GameData&);
		GameData& operator=(const GameData&);

		void SetInputHandler(std::shared_ptr<IGameLogic> pInputHandler);
		void SetPhysicsEngine(std::shared_ptr<Physics::IPhysicsEngine> pPhysics);
		void SetInputStateHandler(std::shared_ptr<Display::IInputState> pInputState);
		void SetDisplayComponent(std::shared_ptr<Display::IDisplay> pDisplay);
		void SetTimer(std::shared_ptr<ITimer> pTimer);
		void SetEventManager(std::shared_ptr<Events::IEventManager> pManager);

		std::unique_ptr<GameDataImpl> m_pData;
	};
}