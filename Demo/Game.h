#ifndef GAME_H
#define GAME_H

#include "enginefwd.h"

namespace GameEngine
{
	class Game
	{
	public:
		Game();
		~Game();
		Game(Game& game);
		int Run();
	private:
		GameImpl *pImpl;
	};
}

#endif