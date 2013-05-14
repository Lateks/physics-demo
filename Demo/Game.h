#pragma once
#include "enginefwd.h"

namespace GameEngine
{
	struct GameImpl;

	class Game
	{
	public:
		Game();
		virtual ~Game();
		bool Initialize(const IGameLogicFactory& gameLogicFactory,
			const Display::IDisplayFactory& displayFactory,
			const Physics::IPhysicsEngineFactory& physicsFactory);
		bool Run() const;
	private:
		Game(const Game&);
		std::unique_ptr<GameImpl> m_pImpl;
	};
}