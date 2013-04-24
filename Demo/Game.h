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
		std::shared_ptr<GameData> m_pData;
		void HandleInputs();
		void ThrowCube(Vec3& throwTowards);
	};
}

#endif