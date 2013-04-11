#include "Game.h"
#include "GameData.h"
#include "Sphere.h"
#include "Cube.h"
#include "IRenderer.h"
#include "RenderingEngineFactories.h"
#include "Vec3.h"
#include <irrlicht.h>
#include <iostream>

namespace GameEngine
{
	using LinearAlgebra::Vec3;
	using Display::IRenderer;

	void SetupGameActors(GameData *game)
	{
		/*
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
		*/
	}

	Game::Game()
	{
		IRenderer *renderer = Display::CreateRenderer().release();
		if (!renderer)
		{
			std::cerr << "Failed to create rendering device." << std::cerr;
			return;
		}

		if (!renderer->SetupAndOpenWindow(800, 600,
			Display::DRIVER_TYPE::OPEN_GL, Display::CAMERA_TYPE::FPS))
		{
			std::cerr << "Failed to open OpenGL device." << std::cerr;
			return;
		}

		renderer->SetCameraPosition(Vec3(-25, 60, -50));
		renderer->SetCameraTarget(Vec3(17, 3, 0));
		m_pData = GameData::getInstance();
		m_pData->SetRenderer(renderer);
		// TODO: set physics engine

		SetupGameActors(m_pData);
	}

	Game::~Game()
	{
		delete m_pData;
	}

	Game::Game(Game& game)
	{
		if (this != &game)
		{
			delete m_pData;
			m_pData = game.m_pData;
			game.m_pData = nullptr;
		}
	}

	int Game::Run()
	{
		IRenderer *renderer = m_pData->GetRenderer();
		if (!renderer)
			return 1;

		unsigned int timeBegin = m_pData->CurrentTime();
		unsigned int timeEnd;
		float frameDeltaSec = 1.0f/60;
		while (renderer->Running())
		{
			if (renderer->WindowActive())
			{
				// physics->VUpdateSimulation(frameDeltaSec)
				// physics->VSyncScene()
				m_pData->MoveAllActors(frameDeltaSec);
				renderer->DrawScene();
			}
			else
				renderer->YieldDevice();
			timeEnd = m_pData->CurrentTime();
			frameDeltaSec = 1.0f * (timeEnd - timeBegin) / 1000.0f;
			timeBegin = timeEnd;
		}

		return 0;
	}
}