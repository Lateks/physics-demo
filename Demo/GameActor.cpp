#include "stdafx.h"
#include "GameActor.h"
#include <irrlicht.h>

namespace
{
	unsigned int ID = 1;
}

GameActor::GameActor(irr::scene::ISceneNode *model)
	: pModel(model)
{
	actorId = ID++;
}

GameActor::~GameActor()
{
	delete pModel;
}