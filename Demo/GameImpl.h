#ifndef GAME_IMPL_H
#define GAME_IMPL_H

#include "enginefwd.h"
#include <map>

namespace GameEngine
{
	struct GameImpl
	{
		GameImpl(Display::IrrlichtRenderer *pRendererNew)
			: pRenderer(pRendererNew) { }
		~GameImpl();
		unsigned int CurrentTime();
		void MoveAllActors(float scale);

		Display::IrrlichtRenderer *pRenderer;
		// ICollisionEngine *collisionEngine;
		std::map<unsigned int, GameActor*> actors;
	};
}

#endif