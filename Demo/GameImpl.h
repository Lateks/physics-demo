#ifndef GAME_IMPL_H
#define GAME_IMPL_H

#include <map>

// class ICollisionEngine;
struct IrrlichtRenderer;
class GameActor;

struct GameImpl
{
	GameImpl(IrrlichtRenderer *pRendererNew)
		: pRenderer(pRendererNew) { }
	~GameImpl();
	unsigned int CurrentTime();
	void MoveAllActors(float scale);

	IrrlichtRenderer *pRenderer;
	// ICollisionEngine *collisionEngine;
	std::map<unsigned int, GameActor*> actors;
};

#endif