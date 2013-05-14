#include "GameActor.h"
#include "WorldTransformComponent.h"
#include "Vec3.h"
#include <iostream>
#include <string>

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
		std::wstring m_actorName;
		WorldTransformComponent m_pTransform;
	};

	GameActor::GameActor(std::wstring name)
		: m_pData(new GameActorData())
	{
		m_pData->m_actorId = ++ID; // Note: numbering starts from 1.
		m_pData->m_actorName = name;
	}

	GameActor::GameActor(Vec3& startPosition, std::wstring name)
		: m_pData(new GameActorData())
	{
		m_pData->m_actorId = ++ID;
		m_pData->m_pTransform.SetPosition(startPosition);
		m_pData->m_actorName = name;
	}

	GameActor::~GameActor() { }

	ActorID GameActor::GetID()
	{
		return m_pData->m_actorId;
	}

	std::wstring GameActor::GetName()
	{
		return m_pData->m_actorName;
	}

	WorldTransformComponent& GameActor::GetWorldTransform()
	{
		return m_pData->m_pTransform;
	}
}