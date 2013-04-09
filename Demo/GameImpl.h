#ifndef GAME_IMPL_H
#define GAME_IMPL_H

#include <map>

namespace GameEngine
{
	// class ICollisionEngine;
	namespace Display
	{
		struct IrrlichtRenderer;
	}
	class GameActor;

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