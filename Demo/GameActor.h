#ifndef GAME_ACTOR_H
#define GAME_ACTOR_H

#include "enginefwd.h"
#include <memory>

namespace GameEngine
{
	class GameActor
	{
	public:
		GameActor();
		GameActor(Vec3& startPosition);
		virtual ~GameActor() { };
		unsigned int GetID() { return actorId; }
		void SetWorldTransform(WorldTransformComponent *trans);
		std::weak_ptr<WorldTransformComponent> GetWorldTransform();
	private:
		ActorID actorId;
		/* The transform component is stored separately from the
		 * 3D model because it can be used by several engine components
		 * (such as the physics engine and AI engine).
		 */
		std::shared_ptr<WorldTransformComponent> m_pTransform;
	};
}


#endif