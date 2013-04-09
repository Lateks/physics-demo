#ifndef GAME_ACTOR_H
#define GAME_ACTOR_H

#include "enginefwd.h"
#include <irrlicht.h>
#include <memory>

namespace GameEngine
{
	// TODO: make this an interface that is independent of Irrlicht
	class GameActor
	{
	public:
		GameActor(irr::scene::ISceneNode *model);
		virtual ~GameActor();
		unsigned int GetID() { return actorId; }
		void Move(float seconds);
		void SetPosition(const irr::core::vector3df& newPos);
		irr::scene::ISceneNode *pModel;
		irr::core::vector3df movementNormal;
		float movementSpeed;
		// CollisionShape *pCollShape;
	private:
		ActorID actorId;
	};
}


#endif