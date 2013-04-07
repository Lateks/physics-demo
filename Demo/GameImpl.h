#ifndef GAME_IMPL_H
#define GAME_IMPL_H

#include <map>

// class ICollisionEngine;
struct IrrlichtRenderer;
struct GameActor;

struct GameImpl
{
	GameImpl(IrrlichtRenderer *pRendererNew)
		: pRenderer(pRendererNew) { }
	~GameImpl();
	IrrlichtRenderer *pRenderer;
	// ICollisionEngine *collisionEngine;
	std::map<unsigned int, GameActor*> actors;
};

#endif