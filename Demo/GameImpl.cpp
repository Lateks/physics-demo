#include "GameImpl.h"
#include "GameActor.h"
#include "IrrlichtRenderer.h"
#include <algorithm>
#include <map>

namespace GameEngine
{
	GameImpl::~GameImpl()
	{
		delete pRenderer;
		std::for_each (actors.begin(), actors.end(),
			[] (std::pair<unsigned int, GameActor*> actor) { delete actor.second; });
	}

	unsigned int GameImpl::CurrentTime()
	{
		return pRenderer->pDevice->getTimer()->getTime();
	}

	void GameImpl::MoveAllActors(float scale)
	{
		std::for_each(actors.begin(), actors.end(),
			[scale] (std::pair<unsigned int, GameActor*> actor)
		{
			if (actor.second != nullptr)
			{
				actor.second->Move(scale);
			}
		}
		);
	}
}