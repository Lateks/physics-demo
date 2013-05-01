#ifndef GAME_H
#define GAME_H

#include "enginefwd.h"

namespace GameEngine
{
	class Game
	{
	public:
		Game();
		virtual ~Game();
		Game(Game&& game);
		int Run();
	private:
		std::shared_ptr<GameData> m_pData;
	};
}

#endif