#include "DemoInputHandler.h"
#include "WorldTransformComponent.h"
#include "IPhysicsEngine.h"
#include "IEventManager.h"
#include "Events.h"
#include "GameData.h"
#include "GameActor.h"
#include "IDisplay.h"
#include "BspLoaderFactory.h"
#include "Vec3.h"
#include <iostream>
#include <cassert>

namespace
{
	unsigned int WOODBOX_TEXTURE;
	unsigned int MUD_TEXTURE;
	unsigned int BEACHBALL_TEXTURE;
}

namespace GameEngine
{
	/* Note: the coordinates given here are all given in a right-handed
	 * coordinate system. The IrrlichtDisplay component converts them to
	 * the left-handed system used by Irrlicht by negating the z component.
	 * (Also note that the concept of handedness does not affect quaternions
	 * used for handling rotations.)
	 */
	void DemoInputHandler::SetupInitialScene(GameData *game)
	{
		Display::IDisplay *pRenderer = game->GetRenderer();
		Physics::IPhysicsEngine *pPhysics = game->GetPhysicsEngine();
		pPhysics->VSetGlobalGravity(Vec3(0, -100.f, 0));

		// Create an actor for the world map to be able to refer to the associated
		// rigid bodies. Note: now that the map itself has an actor and a
		// world transform, the renderer could also use it to determine the
		// position of the map. (TODO?)
		Vec3 mapPosition(-1350, -130, 1400);
		StrongActorPtr world(new GameActor(mapPosition));
		game->AddActor(world);
		pRenderer->LoadMap("..\\assets\\map-20kdm2.pk3", "20kdm2.bsp", mapPosition);
		std::unique_ptr<BspLoader> pBspLoader = CreateBspLoader("..\\assets\\20kdm2.bsp");
		pPhysics->VLoadBspMap(*pBspLoader, world);

		pRenderer->SetCameraPosition(Vec3(50,50,60));
		pRenderer->SetCameraTarget(Vec3(-70,30,60));

		// Load textures.
		MUD_TEXTURE = pRenderer->LoadTexture("..\\assets\\cracked_mud.jpg");
		WOODBOX_TEXTURE = pRenderer->LoadTexture("..\\assets\\woodbox2.jpg");

		// Setup actors and their graphical and physical representations.
		StrongActorPtr ball(new GameActor(Vec3(0, 50, 60)));
		game->AddActor(ball);
		pRenderer->AddSphereSceneNode(10.f, ball, MUD_TEXTURE);
		pPhysics->VAddSphere(10.f, ball, "Titanium", "Bouncy");

		StrongActorPtr cube(new GameActor(Vec3(0, 80, 60)));
		game->AddActor(cube);
		pRenderer->AddCubeSceneNode(25.f, cube, WOODBOX_TEXTURE);
		pPhysics->VAddBox(Vec3(25.f, 25.f, 25.f), cube, "Titanium", "Bouncy");
	}

	void DemoInputHandler::HandleInputs()
	{
		GameData *pGame = GameData::getInstance();
		assert(pGame && pGame->GetInputStateHandler());
		m_currentMouseState = pGame->GetInputStateHandler()->GetMouseState();

		auto pRenderer = pGame->GetRenderer();
		auto pPhysics = pGame->GetPhysicsEngine();
		m_currentCameraState = CameraState(pRenderer->GetCameraPosition(), pRenderer->GetCameraTarget());

		if (RightMouseReleased())
		{
			ThrowCube(pGame->GetRenderer()->GetCameraTarget());
		}
		else if (LeftMousePressed())
		{
			Vec3 pickPoint;
			ActorID pickedActorId = pGame->GetPhysicsEngine()->GetClosestActorHit(
				m_currentCameraState.cameraPos, m_currentCameraState.cameraTarget, pickPoint);
			if (pickedActorId != 0)
			{
				m_pickConstraintId = pPhysics->AddPickConstraint(pickedActorId, pickPoint,
					m_currentCameraState.cameraPos);
				m_pickedActor = pickedActorId;
			}
		}
		else if (LeftMouseReleased() && m_pickedActor != 0)
		{
			pPhysics->RemoveConstraint(m_pickedActor, m_pickConstraintId);
			m_pickedActor = 0;
			m_pickConstraintId = 0;
		}
		else if (CameraMoved())
		{
			auto pEventMgr = pGame->GetEventManager();
			if (pEventMgr)
			{
				std::shared_ptr<Events::IEventData> event(new Events::CameraMoveEvent(pGame->CurrentTimeSec(),
					m_currentCameraState.cameraPos, m_currentCameraState.cameraTarget));
				pEventMgr->QueueEvent(event);
			}
		}

		m_previousMouseState = m_currentMouseState;
		m_previousCameraState = m_currentCameraState;
	}

	void DemoInputHandler::ThrowCube(Vec3& throwTowards)
	{
		GameData *pGame = GameData::getInstance();
		Vec3 cameraPos = pGame->GetRenderer()->GetCameraPosition();

		StrongActorPtr cube(new GameActor(cameraPos));
		std::weak_ptr<WorldTransformComponent> pWeakTransform = cube->GetWorldTransform();
		if (!pWeakTransform.expired())
		{
			std::shared_ptr<WorldTransformComponent> pTransform(pWeakTransform);
			pTransform->SetRotation(pGame->GetRenderer()->GetCameraRotation());
		}
		pGame->AddActor(cube);

		auto renderer = pGame->GetRenderer();
		renderer->AddCubeSceneNode(15.f, cube, WOODBOX_TEXTURE);
		auto physics = pGame->GetPhysicsEngine();
		physics->VAddBox(Vec3(15.f, 15.f, 15.f), cube, "Titanium", "Bouncy");

		Vec3 throwDirection = throwTowards - cameraPos;

		// Also make the object rotate slightly "away from the camera".
		Vec3 rotationAxis = pGame->GetRenderer()->GetCameraRightVector();
		rotationAxis[2] = -rotationAxis[2];

		physics->VSetLinearVelocity(cube->GetID(), throwDirection, 250.f);
		physics->VSetAngularVelocity(cube->GetID(), rotationAxis, 2.5f);
	}

	bool DemoInputHandler::CameraMoved()
	{
		return m_previousCameraState.cameraPos != m_currentCameraState.cameraPos ||
			m_previousCameraState.cameraTarget != m_currentCameraState.cameraTarget;
	}

	bool DemoInputHandler::LeftMousePressed()
	{
		return m_currentMouseState.LeftMouseDown && !m_previousMouseState.LeftMouseDown;
	}

	bool DemoInputHandler::LeftMouseDown()
	{
		return m_currentMouseState.LeftMouseDown && m_previousMouseState.LeftMouseDown;
	}

	bool DemoInputHandler::LeftMouseReleased()
	{
		return !m_currentMouseState.LeftMouseDown && m_previousMouseState.LeftMouseDown;
	}

	bool DemoInputHandler::RightMousePressed()
	{
		return m_currentMouseState.RightMouseDown && !m_previousMouseState.RightMouseDown;
	}

	bool DemoInputHandler::RightMouseDown()
	{
		return m_currentMouseState.RightMouseDown && m_previousMouseState.RightMouseDown;
	}

	bool DemoInputHandler::RightMouseReleased()
	{
		return !m_currentMouseState.RightMouseDown && m_previousMouseState.RightMouseDown;
	}
}