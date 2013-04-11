#include "GameData.h"
#include "GameActor.h"
#include "IRenderer.h"
#include "IPhysicsEngine.h"
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

	// TODO: need some kind of a timer object 
	unsigned int GameData::CurrentTime()
	{
		//return pRenderer->pDevice->getTimer()->getTime();
		return 0;
	}

	void GameData::MoveAllActors(float scale)
	{
		std::for_each(m_actors.begin(), m_actors.end(),
			[scale] (std::pair<unsigned int, GameActor*> actor)
		{
			if (actor.second != nullptr)
			{
				actor.second->Move(scale);
			}
		}
		);
	}

	void GameData::AddActor(GameActor *actor)
	{
		m_actors[actor->GetID()] = actor; 
	}
}