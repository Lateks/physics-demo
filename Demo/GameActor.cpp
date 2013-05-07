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
	struct GameActorData
	{
		GameActorData() : m_actorId(0), m_pTransform() { }
		ActorID m_actorId;
		WorldTransformComponent m_pTransform;
	};

	GameActor::GameActor()
		: m_pData(new GameActorData())
	{
		m_pData->m_actorId = ++ID; // Note: numbering starts from 1.
	}

	GameActor::GameActor(Vec3& startPosition)
		: m_pData(new GameActorData())
	{
		m_pData->m_actorId = ++ID;
		m_pData->m_pTransform.SetPosition(startPosition);
	}

	GameActor::~GameActor() { }

	ActorID GameActor::GetID()
	{
		return m_pData->m_actorId;
	}

	void GameActor::SetWorldTransform(const WorldTransformComponent& trans)
	{
		m_pData->m_pTransform = trans;
	}

	WorldTransformComponent& GameActor::GetWorldTransform()
	{
		return m_pData->m_pTransform;
	}
}