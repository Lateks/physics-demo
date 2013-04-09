#include "stdafx.h"
#include "Game.h"
#include "GameImpl.h"
#include "Sphere.h"
#include "Cube.h"
#include "IrrlichtRenderer.h"
#include <irrlicht.h>
#include <iostream>

namespace GameEngine
{
	using irr::core::vector3df;
	using Display::IrrlichtRenderer;

	void SetupCamera(irr::scene::ICameraSceneNode *pCamera)
	{
		pCamera->setPosition(vector3df(-25, 60, -50));
		pCamera->setTarget(vector3df(17, 3, 0));
		pCamera->updateAbsolutePosition();
	}

	void SetupGameActors(GameImpl *game)
	{
		auto mudTexture = game->pRenderer->pDriver->getTexture("..\\assets\\cracked_mud.jpg");
		auto woodBoxTexture = game->pRenderer->pDriver->getTexture("..\\assets\\woodbox2.jpg");
		auto headCrabTexture = game->pRenderer->pDriver->getTexture("..\\assets\\headcrabsheet.tga");

		Sphere *sphere1 = new Sphere(7.5f, game->pRenderer->pSmgr);
		sphere1->SetPosition(vector3df(25, 0, 20));
		sphere1->pModel->setMaterialTexture(0, mudTexture);

		Sphere *sphere2 = new Sphere(4.5f, game->pRenderer->pSmgr);
		sphere2->SetPosition(vector3df(-25, 0, 20));
		sphere2->pModel->setMaterialTexture(0, mudTexture);
		sphere2->movementNormal = vector3df(1,0,0);
		sphere2->movementSpeed = 7.0f;

		Cube *box1 = new Cube(10.0f, game->pRenderer->pSmgr);
		box1->SetPosition(vector3df(0, 0, 0));
		box1->pModel->setMaterialTexture(0, woodBoxTexture);
		box1->movementNormal = vector3df(1, 0, 1).normalize();
		box1->movementSpeed = 10.0f;

		auto headCrab = game->pRenderer->pSmgr->getMesh("..\\assets\\headcrabclassic.obj");
		auto headCrabNode = game->pRenderer->pSmgr->addAnimatedMeshSceneNode(headCrab);
		headCrabNode->setMaterialFlag(irr::video::EMF_LIGHTING, false);
		headCrabNode->setMaterialTexture(0, headCrabTexture);

		game->actors[sphere1->GetID()] = sphere1;
		game->actors[sphere2->GetID()] = sphere2;
		game->actors[box1->GetID()] = box1;
	}

	Game::Game()
	{
		IrrlichtRenderer *renderer = new IrrlichtRenderer();

		if (!renderer->SetupAndOpenWindow(800, 600, irr::video::EDT_OPENGL))
		{
			std::cerr << "Failed to open OpenGL device." << std::cerr;
			return;
		}

		SetupCamera(renderer->pCamera);
		pImpl = new GameImpl(renderer);
		SetupGameActors(pImpl);
		// TODO: setup collisions
	}

	Game::~Game()
	{
		delete pImpl;
	}

	Game::Game(Game& game)
	{
		delete pImpl;
		pImpl = game.pImpl;
		game.pImpl = nullptr;
	}

	int Game::Run()
	{
		if (pImpl->pRenderer == nullptr)
			return 1;

		unsigned int timeBegin = pImpl->CurrentTime();
		unsigned int timeEnd;
		float frameDeltaSec = 1.0f/60;
		while (pImpl->pRenderer->pDevice->run())
		{
			if (pImpl->pRenderer->pDevice->isWindowActive())
			{
				// TODO: test collisions
				pImpl->MoveAllActors(frameDeltaSec);
				pImpl->pRenderer->DrawScene(false, false);
			}
			else
				pImpl->pRenderer->pDevice->yield();
			timeEnd = pImpl->CurrentTime();
			frameDeltaSec = 1.0f * (timeEnd - timeBegin) / 1000.0f;
			timeBegin = timeEnd;
		}

		return 0;
	}
}