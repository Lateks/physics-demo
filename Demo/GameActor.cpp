#include "stdafx.h"
#include "GameActor.h"
#include <irrlicht.h>
#include <iostream>

using irr::core::vector3df;

namespace
{
	unsigned int ID = 1;
}

namespace GameEngine
{
	GameActor::GameActor(irr::scene::ISceneNode *model)
		: pModel(model), movementNormal(0)
	{
		pModel->setMaterialFlag(irr::video::EMF_LIGHTING, false);
		actorId = ID++;
	}

	GameActor::~GameActor()
	{
		pModel->drop();
	}

	void GameActor::SetPosition(const vector3df& newPos)
	{
		pModel->setPosition(newPos);
		pModel->updateAbsolutePosition();
	}

	void GameActor::Move(float seconds)
	{
		// For simplicity, GameActor ISceneNodes never have parent nodes,
		// so their relative position is also their absolute position.
		vector3df newPosition = pModel->getAbsolutePosition() + movementNormal *  seconds * movementSpeed;
		pModel->setPosition(newPosition);
	}
}