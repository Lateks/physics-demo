#include "GameActor.h"
#include "WorldTransformComponent.h"
#include <iostream>

namespace
{
	unsigned int ID = 1;
}

namespace GameEngine
{
	GameActor::GameActor()
	{
		actorId = ID++;
	}

	void GameActor::SetWorldTransform(WorldTransformComponent *trans)
	{
		m_pTransform.reset(trans);
	}
	std::weak_ptr<WorldTransformComponent> GameActor::GetWorldTransform()
	{
		return std::weak_ptr<WorldTransformComponent>(m_pTransform);
	}
}