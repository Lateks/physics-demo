#pragma once

#include "enginefwd.h"
#include <memory>

namespace GameEngine
{
	struct GameDataImpl;

	class GameData
	{
	private:
		GameData();
		std::unique_ptr<GameDataImpl> m_pData;
	public:
		static std::shared_ptr<GameData> GetInstance();
		~GameData();

		float CurrentTimeSec();

		void AddActor(std::weak_ptr<GameActor> pActor);
		std::shared_ptr<GameActor> GetActor(ActorID id);

		void SetInputHandler(std::shared_ptr<IGameLogic> pInputHandler);
		std::shared_ptr<IGameLogic> GetInputHandler() const;

		void SetPhysicsEngine(std::shared_ptr<Physics::IPhysicsEngine> pPhysics);
		std::shared_ptr<Physics::IPhysicsEngine> GetPhysicsEngine() const;

		void SetInputStateHandler(std::shared_ptr<Display::IInputState> pInputState);
		std::shared_ptr<Display::IInputState> GetInputStateHandler() const;

		void SetDisplayComponent(std::shared_ptr<Display::IDisplay> pDisplay);
		std::shared_ptr<Display::IDisplay> GetDisplayComponent() const;

		void SetTimer(std::shared_ptr<ITimer> pTimer);
		std::shared_ptr<ITimer> GetTimer() const;

		void SetEventManager(std::shared_ptr<Events::IEventManager> pManager);
		std::shared_ptr<Events::IEventManager> GetEventManager() const;
	};
}