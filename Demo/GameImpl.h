#ifndef GAME_IMPL_H
#define GAME_IMPL_H

#include <map>

// class ICollisionEngine;
class BasicIrrlichtRenderer;
struct GameActor;

struct GameImpl
{
	GameImpl(BasicIrrlichtRenderer *pRenderer)
		: renderer(pRenderer) { }
	~GameImpl();
	BasicIrrlichtRenderer *renderer;
	// ICollisionEngine *collisionEngine;
	std::map<unsigned int, GameActor*> actors;
};

#endif