#include "GameData.h"
#include "GameActor.h"
#include "IDisplay.h"
#include "IPhysicsEngine.h"
#include "IEventManager.h"
#include "ITimer.h"
#include <algorithm>
#include <map>

namespace GameEngine
{
	GameData *GameData::instance = nullptr;

	GameData::~GameData()
	{
		delete m_pRenderer;
		delete m_pPhysicsEngine;
		delete m_pEvents;
		m_actors.clear();
		GameData::instance = nullptr;
	}

	float GameData::CurrentTimeSec()
	{
		return m_pTimer->GetTimeMs() / 1000.0f;
	}

	void GameData::AddActor(WeakActorPtr pActor)
	{
		if (pActor.expired())
			return;

		StrongActorPtr pStrongActor(pActor);
		m_actors[pStrongActor->GetID()] = pStrongActor; 
	}
}