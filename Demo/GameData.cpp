#include "GameData.h"
#include "GameActor.h"
#include "IRenderer.h"
#include "IPhysicsEngine.h"
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
		std::for_each (m_actors.begin(), m_actors.end(),
			[] (std::pair<unsigned int, GameActor*> actor) { delete actor.second; });
		GameData::instance = nullptr;
	}

	unsigned int GameData::CurrentTime()
	{
		return m_pTimer->GetTime();
	}

	void GameData::AddActor(GameActor *actor)
	{
		m_actors[actor->GetID()] = actor; 
	}
}