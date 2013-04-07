#include "stdafx.h"
#include "GameImpl.h"
#include "GameActor.h"
#include "IrrlichtRenderer.h"
#include <algorithm>
#include <map>

GameImpl::~GameImpl()
{
	delete pRenderer;
	std::for_each (actors.begin(), actors.end(),
		[] (std::pair<unsigned int, GameActor*> actor) { delete actor.second; });
}