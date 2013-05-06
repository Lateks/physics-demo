#pragma once

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
		std::shared_ptr<WorldTransformComponent> GetWorldTransform();
	private:
		ActorID actorId;
		/* The transform component is stored separately from the
		 * 3D model because it can be used by several engine components
		 * (such as the physics engine and AI engine).
		 */
		std::shared_ptr<WorldTransformComponent> m_pTransform;
	};
}