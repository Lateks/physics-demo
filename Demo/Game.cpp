#include "Game.h"
#include "GameActor.h"
#include "GameData.h"
#include "IRenderer.h"
#include "TimerFactories.h"
#include "IEventManager.h"
#include "EventManager.h" // TODO: make a factory method for these.
#include "RenderingEngineFactories.h"
#include "IPhysicsEngine.h"
#include "PhysicsEngineFactories.h"
#include "BspLoaderFactory.h"
#include "Vec3.h"
#include <irrlicht.h>
#include <iostream>
#include <memory>

namespace GameEngine
{
	using Display::IRenderer;
	using PhysicsEngine::IPhysicsEngine;
	using Events::IEventManager;
	using Events::EventManager;

	/* Note: the coordinates given here are all given in a right-handed
	 * coordinate system. The IrrlichtRenderer component converts them to
	 * the left-handed system used by Irrlicht by negating the z component.
	 * (Also note that the concept of handedness does not affect quaternions
	 * used for handling rotations.)
	 */
	void SetupInitialScene(GameData *game)
	{
		IRenderer *renderer = game->GetRenderer();
		IPhysicsEngine *physics = game->GetPhysicsEngine();

		// Create an actor for the world map to be able to refer to the associated
		// rigid bodies. Note: now that the map itself has an actor and a
		// world transform, the renderer could also use it to determine the
		// position of the map.
		Vec3 mapPosition(-1350, -130, 1400);
		StrongActorPtr world(new GameActor(mapPosition));
		game->AddActor(world);
		renderer->LoadMap("..\\assets\\map-20kdm2.pk3", "20kdm2.bsp", mapPosition);
		std::unique_ptr<BspLoader> pBspLoader = CreateBspLoader("..\\assets\\20kdm2.bsp");
		physics->VAddBspMap(*pBspLoader, world);

		renderer->SetCameraPosition(Vec3(50,50,60));
		renderer->SetCameraTarget(Vec3(-70,30,60));

		unsigned int mudTexture = renderer->LoadTexture("..\\assets\\cracked_mud.jpg");
		//unsigned int woodBoxTexture = renderer->LoadTexture("..\\assets\\woodbox2.jpg");

		StrongActorPtr ball(new GameActor(Vec3(0, 50, 60)));
		game->AddActor(ball);
		renderer->AddSphereSceneNode(7.5f, ball->GetID(), mudTexture);
		physics->VAddSphere(7.5f, ball, "styrofoam", "Bouncy");

		/*
		unsigned int headCrabTexture = renderer->LoadTexture("..\\assets\\headcrabsheet.tga");

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
		// Setup rendering component.
		std::unique_ptr<IRenderer> renderer(Display::CreateRenderer());
		if (!renderer.get())
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

		m_pData = GameData::getInstance();
		m_pData->SetRenderer(renderer.release());

		// Setup timer.
		std::unique_ptr<ITimer> timer(GetTimer());
		if (!timer.get())
		{
			std::cerr << "Failed to create a timer." << std::endl;
			return;
		}
		m_pData->setTimer(timer.release());

		// Setup event manager.
		std::unique_ptr<IEventManager> pEventManager(new EventManager());
		if (!pEventManager.get())
		{
			std::cerr << "Failed to create an event manager." << std::endl;
			return;
		}
		m_pData->SetEventManager(pEventManager.release());

		// Setup physics.
		std::unique_ptr<PhysicsEngine::IPhysicsEngine> physics(
			PhysicsEngine::CreatePhysicsEngine());
		if (!physics.get())
		{
			std::cerr << "Failed to initialize physics engine." << std::endl;
			return;
		}
		m_pData->SetPhysicsEngine(physics.release());
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
		SetupInitialScene(m_pData);

		IRenderer *renderer = m_pData->GetRenderer();
		IPhysicsEngine *physics = m_pData->GetPhysicsEngine();
		IEventManager *events = m_pData->GetEventManager();
		if (!renderer || !physics || !events)
			return 1;

		float timeBegin = m_pData->CurrentTimeSec();
		float timeEnd;
		float frameDeltaSec = 1.0f/60;
		while (renderer->Running())
		{
			if (renderer->WindowActive())
			{
				physics->VUpdateSimulation(frameDeltaSec);
				physics->VSyncScene();
				events->DispatchEvents();
				// TODO: handle inputs
				renderer->DrawScene();
			}
			else
			{
				renderer->YieldDevice();
			}
			timeEnd = m_pData->CurrentTimeSec();
			frameDeltaSec = timeEnd - timeBegin;
			timeBegin = timeEnd;
		}

		return 0;
	}
}