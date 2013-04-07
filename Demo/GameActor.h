#ifndef GAME_ACTOR_H
#define GAME_ACTOR_H

namespace irr
{
	namespace scene
	{
		class ISceneNode;
	}
}

class GameActor
{
public:
	GameActor(irr::scene::ISceneNode *model);
	~GameActor();
	unsigned int GetID() { return actorId; }
	irr::scene::ISceneNode *pModel;
	// CollisionShape *pCollShape;
private:
	unsigned int actorId;
};

#endif