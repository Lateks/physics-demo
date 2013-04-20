#include "Game.h"
#include "GameActor.h"
#include "GameData.h"
#include "IDisplay.h"
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
	using Display::IDisplay;
	using Physics::IPhysicsEngine;
	using Events::IEventManager;
	using Events::EventManager;

	/* Note: the coordinates given here are all given in a right-handed
	 * coordinate system. The IrrlichtDisplay component converts them to
	 * the left-handed system used by Irrlicht by negating the z component.
	 * (Also note that the concept of handedness does not affect quaternions
	 * used for handling rotations.)
	 */
	void SetupInitialScene(GameData *game)
	{
		IDisplay *renderer = game->GetRenderer();
		IPhysicsEngine *physics = game->GetPhysicsEngine();
		physics->VSetGlobalGravity(Vec3(0, -100.f, 0));

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
		unsigned int woodBoxTexture = renderer->LoadTexture("..\\assets\\woodbox2.jpg");

		StrongActorPtr ball(new GameActor(Vec3(0, 50, 60)));
		game->AddActor(ball);
		renderer->AddSphereSceneNode(10.f, ball->GetID(), mudTexture);
		physics->VAddSphere(10.f, ball, "Vinyl", "Bouncy");

		StrongActorPtr cube(new GameActor(Vec3(0, 80, 60)));
		game->AddActor(cube);
		renderer->AddCubeSceneNode(15.f, cube->GetID(), woodBoxTexture);
		physics->VAddBox(Vec3(15.f, 15.f, 15.f), cube, "Titanium", "Bouncy");
	}

	Game::Game()
	{
		// Setup rendering component.
		std::unique_ptr<IDisplay> renderer(Display::CreateRenderer());
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
		std::unique_ptr<Physics::IPhysicsEngine> physics(
			Physics::CreatePhysicsEngine());
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

		IDisplay *renderer = m_pData->GetRenderer();
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