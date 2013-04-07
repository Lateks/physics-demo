#ifndef GAME_ACTOR_H
#define GAME_ACTOR_H

#include <irrlicht.h>

class GameActor
{
public:
	GameActor(irr::scene::ISceneNode *model);
	virtual ~GameActor();
	unsigned int GetID() { return actorId; }
	void Move(float seconds);
	irr::scene::ISceneNode *pModel;
	irr::core::vector3df movementNormal;
	float movementSpeed;
	// CollisionShape *pCollShape;
private:
	unsigned int actorId;
};

#endif