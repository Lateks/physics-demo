#ifndef GAME_ACTOR_H
#define GAME_ACTOR_H

#include <irrlicht.h>

namespace GameEngine
{
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
		unsigned int actorId;
	};
}

#endif