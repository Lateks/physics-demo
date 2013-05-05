#include "GameActor.h"
#include "WorldTransformComponent.h"
#include "Vec3.h"
#include <iostream>

namespace
{
	unsigned int ID = 0;
}

namespace GameEngine
{
	GameActor::GameActor()
		: m_pTransform(new WorldTransformComponent())
	{
		actorId = ++ID; // Note: numbering starts from 1.
	}

	GameActor::GameActor(Vec3& startPosition)
		: m_pTransform(new WorldTransformComponent())
	{
		actorId = ++ID;
		m_pTransform->SetPosition(startPosition);
	}

	void GameActor::SetWorldTransform(WorldTransformComponent *trans)
	{
		m_pTransform.reset(trans);
	}

	std::shared_ptr<WorldTransformComponent> GameActor::GetWorldTransform()
	{
		return m_pTransform;
	}
}