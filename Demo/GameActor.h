#pragma once

#include "enginefwd.h"
#include <memory>

namespace GameEngine
{
	struct GameActorData;

	class GameActor
	{
	public:
		GameActor();
		GameActor(Vec3& startPosition);
		virtual ~GameActor();
		ActorID GetID();
		void SetWorldTransform(const WorldTransformComponent& trans);
		WorldTransformComponent& GetWorldTransform();
	private:
		std::unique_ptr<GameActorData> m_pData;
	};
}