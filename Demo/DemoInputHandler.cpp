#include "DemoInputHandler.h"
#include "WorldTransformComponent.h"
#include "IPhysicsEngine.h"
#include "MessagingWindow.h"
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
	void DemoInputHandler::SetupInitialScene()
	{
		auto pGame = GameData::GetInstance();
		Display::IDisplay *pRenderer = pGame->GetRenderer();
		Physics::IPhysicsEngine *pPhysics = pGame->GetPhysicsEngine();
		Events::IEventManager *pEventMgr = pGame->GetEventManager();
		std::shared_ptr<Display::MessagingWindow> pMessages = pRenderer->GetMessageWindow();
		pMessages->SetFont("..\\assets\\fontlucida.png");
		pMessages->SetVisible(true);
		pMessages->SetWidth(600);

		// Create an actor for the world map to be able to refer to the associated
		// rigid bodies. Note: now that the map itself has an actor and a
		// world transform, the renderer could also use it to determine the
		// position of the map. (TODO?)
		Vec3 mapPosition(-1350, -130, 1400);
		StrongActorPtr world(new GameActor(mapPosition));
		pGame->AddActor(world);
		pRenderer->LoadMap("..\\assets\\map-20kdm2.pk3", "20kdm2.bsp", mapPosition);
		std::unique_ptr<BspLoader> pBspLoader = CreateBspLoader("..\\assets\\20kdm2.bsp");
		pPhysics->VLoadBspMap(*pBspLoader, world);

		pRenderer->SetCameraPosition(Vec3(80,50,60));
		pRenderer->SetCameraTarget(Vec3(-70,30,60));

		// Load textures.
		MUD_TEXTURE = pRenderer->LoadTexture("..\\assets\\cracked_mud.jpg");
		WOODBOX_TEXTURE = pRenderer->LoadTexture("..\\assets\\woodbox2.jpg");

		// Setup actors and their graphical and physical representations.
		StrongActorPtr ball(new GameActor(Vec3(0, 50, 60)));
		pGame->AddActor(ball);
		pRenderer->AddSphereSceneNode(10.f, ball, MUD_TEXTURE);
		pPhysics->VAddSphere(10.f, ball, "Titanium", "Bouncy");

		StrongActorPtr cube(new GameActor(Vec3(0, 80, 60)));
		pGame->AddActor(cube);
		pRenderer->AddCubeSceneNode(25.f, cube, WOODBOX_TEXTURE);
		pPhysics->VAddBox(Vec3(25.f, 25.f, 25.f), cube, "manganese", "Normal");

		// Add a trigger node, rendered as a wireframe cube. (The IrrlichtDisplay
		// assumes you want a wireframe when no texture is given.)
		StrongActorPtr trigger(new GameActor(Vec3(-100.f, 125.f, 450.f)));
		pGame->AddActor(trigger);
		pRenderer->AddCubeSceneNode(125.f, trigger, 0);
		pPhysics->VCreateTrigger(trigger, 125.f);

		// Create an event handler to print out messages when a trigger event is detected.
		Events::EventHandlerPtr eventPrinter(new std::function<void(Events::EventPtr)>
			([pMessages, this] (Events::EventPtr event)
		{
			this->PrintTriggerEvent(pMessages, event);
		}));
		pEventMgr->RegisterHandler(Events::EventType::ENTER_TRIGGER, eventPrinter);
		pEventMgr->RegisterHandler(Events::EventType::EXIT_TRIGGER, eventPrinter);
	}

	void DemoInputHandler::PrintTriggerEvent(std::shared_ptr<Display::MessagingWindow> pMessages,
		Events::EventPtr event)
	{
		Events::TriggerEvent *pEvent = dynamic_cast<Events::TriggerEvent*>(event.get());
		std::wstringstream message;
		message << L"Actor " << pEvent->GetActorId();
		if (event->GetEventType() == Events::EventType::ENTER_TRIGGER)
		{
				message << L" entered the trigger.";
		}
		else if (event->GetEventType() == Events::EventType::EXIT_TRIGGER)
		{
			message << L" exited the trigger.";
		}
		pMessages->AddMessage(message.str());
	}

	void DemoInputHandler::HandleInputs()
	{
		auto pGame = GameData::GetInstance();
		assert(pGame && pGame->GetInputStateHandler());
		m_currentMouseState = pGame->GetInputStateHandler()->GetMouseState();

		auto pRenderer = pGame->GetRenderer();
		auto pPhysics = pGame->GetPhysicsEngine();
		m_currentCameraState = CameraState(pRenderer->GetCameraPosition(), pRenderer->GetCameraTarget());

		if (RightMouseReleased())
		{
			ThrowCube(pGame->GetRenderer()->GetCameraTarget());
		}

		if (LeftMousePressed())
		{
			Vec3 pickPoint;
			Vec3 rayFrom = m_currentCameraState.cameraPos;
			Vec3 rayTo = rayFrom + (m_currentCameraState.cameraTarget-rayFrom).normalized() * 500.f;
			ActorID pickedActorId = pGame->GetPhysicsEngine()->VGetClosestActorHit(
				rayFrom, rayTo, pickPoint);
			std::cerr << pickedActorId << std::endl;
			if (pickedActorId != 0)
			{
				m_pickConstraintId = pPhysics->VAddPickConstraint(pickedActorId, pickPoint,
					m_currentCameraState.cameraPos);
				m_pickedActor = pickedActorId;
			}
		}
		else if (LeftMouseReleased() && m_pickedActor != 0)
		{
			pPhysics->VRemoveConstraint(m_pickedActor, m_pickConstraintId);
			m_pickedActor = 0;
			m_pickConstraintId = 0;
		}

		if (CameraMoved())
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
		auto pGame = GameData::GetInstance();
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

		physics->VSetLinearVelocity(cube->GetID(), throwDirection, 10.f);
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