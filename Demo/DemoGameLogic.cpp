#include "DemoGameLogic.h"
#include "IInputState.h"
#include "IEventManager.h"
#include "IPhysicsEngine.h"
#include "MessagingWindow.h"
#include "IEventManager.h"
#include "Events.h"
#include "IDisplay.h"
#include "GameData.h"

#include "Vec3.h"
#include "Vec4.h"

#include "GameActor.h"
#include "WorldTransformComponent.h"

#include "BspLoaderFactory.h"

#include <iostream>
#include <cassert>

using namespace GameEngine;

namespace
{
	unsigned int WOODBOX_TEXTURE;
	unsigned int MUD_TEXTURE;
	unsigned int BEACHBALL_TEXTURE;

	const float HL_UPDATE_INTERVAL = 1.f/60;
}

namespace Demo
{
	struct CameraState
	{
		CameraState() { }
		CameraState(Vec3& pos, Vec3& target)
			: cameraPos(pos), cameraTarget(target) { };
		Vec3 cameraPos;
		Vec3 cameraTarget;
	};

	struct DemoGameLogicData
	{
		Display::IInputState::MouseState m_previousMouseState;
		Display::IInputState::MouseState m_currentMouseState;
		CameraState m_previousCameraState;
		CameraState m_currentCameraState;

		ActorID m_pickedActor;
		unsigned int m_pickConstraintId;
		float m_updateTimer;

		inline bool CameraMoved();
		inline bool LeftMousePressed();
		inline bool LeftMouseDown();
		inline bool LeftMouseReleased();
		inline bool RightMousePressed();
		inline bool RightMouseDown();
		inline bool RightMouseReleased();
		void UpdateHighlight(float deltaSec);
		ActorID PickTarget(Vec3& pickPoint);
	};
	
	void AddHighlight(ActorID actorId)
	{
		auto pDisplay = GameData::GetInstance()->GetDisplayComponent();
		if (actorId && pDisplay)
		{
			pDisplay->VSetSceneNodeShininess(actorId, 10.f);
			pDisplay->VSetSceneNodeLightColors(actorId, RGBAColor::Blue, RGBAColor::Blue, RGBAColor::Blue);
		}
	}

	void RemoveHighlight(ActorID actorId)
	{
		auto pDisplay = GameData::GetInstance()->GetDisplayComponent();
		if (actorId && pDisplay)
		{
			pDisplay->VSetSceneNodeShininess(actorId, 0.f);
			pDisplay->VSetSceneNodeLightColors(actorId, RGBAColor::White, RGBAColor::White, RGBAColor::White);
		}
	}

	ActorID DemoGameLogicData::PickTarget(Vec3& pickPoint)
	{
		auto pGameData = GameData::GetInstance();
		auto pRaycaster = pGameData->GetPhysicsEngine();
		ActorID pickedActorId = 0;
		if (pRaycaster)
		{
			Vec3 rayFrom = m_currentCameraState.cameraPos;
			Vec3 targetVector = (m_currentCameraState.cameraTarget-rayFrom).normalized() * 500.f;
			Vec3 rayTo = rayFrom + targetVector;
			pickedActorId = pGameData->GetPhysicsEngine()->VGetClosestActorHit(
				rayFrom, rayTo, pickPoint);
		}
		return pickedActorId;
	}

	void DemoGameLogicData::UpdateHighlight(float deltaSec)
	{
		auto pGameData = GameData::GetInstance();
		m_updateTimer += deltaSec;
		if (m_updateTimer > HL_UPDATE_INTERVAL)
		{
			if (!m_pickConstraintId)
			{
				Vec3 pickPoint;
				ActorID pickedActorId = PickTarget(pickPoint);
				if (pickedActorId != m_pickedActor)
				{
					if (pickedActorId)
					{
						AddHighlight(pickedActorId);
					}
					if (m_pickedActor)
					{
						RemoveHighlight(m_pickedActor);
					}
				}
				m_pickedActor = pickedActorId;
			}

			m_updateTimer = 0.f;
		}
	}

	void PrintTriggerEvent(std::shared_ptr<Display::MessagingWindow> pMessages,
		Events::EventPtr event)
	{
		std::shared_ptr<Events::TriggerEvent> pEvent =
			std::dynamic_pointer_cast<Events::TriggerEvent>(event);
		std::wstringstream message;
		if (event->VGetEventType() == Events::EventType::ENTER_TRIGGER)
		{
				message << L"ENTER";
		}
		else if (event->VGetEventType() == Events::EventType::EXIT_TRIGGER)
		{
			message << L"EXIT";
		}
		message << " TRIGGER (actor " << pEvent->GetActorId() << ")";
		pMessages->VAddMessage(message.str());
	}

	void DebugPrintCollisionEvent(Events::EventPtr pEvent)
	{
		bool isCollisionEvent = pEvent->VGetEventType() == Events::EventType::COLLISION_EVENT ||
			pEvent->VGetEventType() == Events::EventType::SEPARATION_EVENT;
		assert(isCollisionEvent);
		if (!isCollisionEvent)
			return;

		auto pCollisionEvent = std::dynamic_pointer_cast<Events::CollisionEvent>(pEvent);
		if (pEvent->VGetEventType() == Events::EventType::COLLISION_EVENT)
			std::cerr << "Collided: ";
		else
			std::cerr << "Separated: ";
		auto collisionPair = pCollisionEvent->GetCollisionPair();
		std::cerr << "actors " << collisionPair.first << " and " << collisionPair.second << std::endl;
	}

	void ThrowCube(Vec3& throwTowards)
	{
		auto pGame = GameData::GetInstance();
		auto pDisplay = pGame->GetDisplayComponent();
		Vec3 cameraPos = pDisplay->VGetCameraPosition();

		auto cube = std::make_shared<GameActor>(cameraPos);
		cube->GetWorldTransform().SetRotation(pDisplay->VGetCameraRotation());
		pGame->AddActor(cube);

		pDisplay->VAddCubeSceneNode(15.f, cube, WOODBOX_TEXTURE, true);
		auto physics = pGame->GetPhysicsEngine();
		physics->VAddBox(cube, Vec3(15.f, 15.f, 15.f), Physics::IPhysicsEngine::PhysicsObjectType::DYNAMIC, "Titanium", "Bouncy");

		Vec3 throwDirection = throwTowards - cameraPos;

		// Make the object rotate slightly "away from the camera".
		Vec3 rotationAxis = pDisplay->VGetCameraRightVector();

		physics->VSetLinearVelocity(cube->GetID(), throwDirection, 175.f);
		physics->VSetAngularVelocity(cube->GetID(), rotationAxis, 2.5f);
	}

	DemoGameLogic::DemoGameLogic() : m_pData(new DemoGameLogicData())
	{
		m_pData->m_updateTimer = 0.f;
		m_pData->m_pickConstraintId = 0;
		m_pData->m_pickedActor = 0;
	}

	DemoGameLogic::~DemoGameLogic() { }

	/* Note: the coordinates given here are all given in a right-handed
	 * coordinate system. The IrrlichtDisplay component converts them to
	 * the left-handed system used by Irrlicht by negating the z component.
	 * (Also note that the concept of handedness does not affect quaternions
	 * used for handling rotations.)
	 */
	bool DemoGameLogic::VSetupInitialScene()
	{
		auto pGame = GameData::GetInstance();
		auto pDisplay = pGame->GetDisplayComponent();
		auto pPhysics = pGame->GetPhysicsEngine();
		auto pEventMgr = pGame->GetEventManager();
		auto pMessages = pDisplay->VGetMessageWindow();
		assert(pDisplay && pPhysics && pEventMgr && pMessages);
		if (!pDisplay || !pPhysics || !pEventMgr || !pMessages)
			return false;

		pMessages->VSetFont("..\\assets\\fontcourier.bmp");
		pMessages->VSetVisible(true);
		pMessages->VSetWidth(600);

		// Create an actor for the world map to be able to refer to the associated
		// rigid bodies.
		Vec3 mapPosition(-1350, -130, -1400);
		auto world = std::make_shared<GameActor>(mapPosition);
		pGame->AddActor(world);
		pDisplay->VLoadMap("..\\assets\\map-20kdm2.pk3", "20kdm2.bsp", mapPosition);
		std::unique_ptr<BspLoader> pBspLoader = CreateBspLoader("..\\assets\\20kdm2.bsp");
		assert(pBspLoader);
		if (!pBspLoader)
			return false;

		pPhysics->VLoadBspMap(*pBspLoader, world, "Tarmac");

		pDisplay->VSetCameraPosition(Vec3(80,40,-60));
		pDisplay->VSetCameraTarget(Vec3(-80,40,-60));

		// Setup lighting.
		pDisplay->VAddLightSceneNode(Vec3(-100, 120, -60),
			RGBAColor(1.f, 1.f, 1.f, 1.f), 500.f);
		pDisplay->VAddLightSceneNode(Vec3(200, 120, -60),
			RGBAColor(1.f, 1.f, 1.f, 1.f), 500.f);
		pDisplay->VAddLightSceneNode(Vec3(-100, 200, -450),
			RGBAColor(1.f, 1.f, 1.f, 1.f), 500.f);
		pDisplay->VAddLightSceneNode(Vec3(150, 200, -450),
			RGBAColor(1.f, 1.f, 1.f, 1.f), 500.f);
		pDisplay->VSetGlobalAmbientLight(RGBAColor(0.1f, 0.1f, 0.15f, 1.f));

		// Load textures.
		MUD_TEXTURE = pDisplay->VLoadTexture("..\\assets\\cracked_mud.jpg");
		WOODBOX_TEXTURE = pDisplay->VLoadTexture("..\\assets\\woodbox2.jpg");

		// Setup actors and their graphical and physical representations.
		auto ball = std::make_shared<GameActor>(Vec3(0, 50, -60));
		pGame->AddActor(ball);
		pDisplay->VAddSphereSceneNode(10.f, ball, MUD_TEXTURE, true);
		pPhysics->VAddSphere(ball, 10.f, Physics::IPhysicsEngine::PhysicsObjectType::DYNAMIC, "Titanium", "Bouncy");

		auto cube = std::make_shared<GameActor>(Vec3(0, 80, -60));
		pGame->AddActor(cube);
		pDisplay->VAddCubeSceneNode(25.f, cube, WOODBOX_TEXTURE, true);
		pPhysics->VAddBox(cube, Vec3(25.f, 25.f, 25.f), Physics::IPhysicsEngine::PhysicsObjectType::DYNAMIC, "manganese", "Normal");

		// Add a trigger node, rendered as a wireframe cube.
		auto trigger = std::make_shared<GameActor>(Vec3(-100, 125, -450));
		pGame->AddActor(trigger);
		pDisplay->VAddCubeSceneNode(125.f, trigger, 0);
		pPhysics->VAddBox(trigger, Vec3(125.f, 125.f, 125.f), Physics::IPhysicsEngine::PhysicsObjectType::TRIGGER);

		// Create an event handler to print out messages when a trigger event is detected.
		Events::EventHandlerPtr eventPrinter(new std::function<void(Events::EventPtr)>
			([pMessages] (Events::EventPtr event)
		{
			PrintTriggerEvent(pMessages, event);
		}));

		Events::EventHandlerPtr debugPrinter(new std::function<void(Events::EventPtr)>
			([] (Events::EventPtr event)
		{
			DebugPrintCollisionEvent(event);
		}));

		pEventMgr->VRegisterHandler(Events::EventType::ENTER_TRIGGER, eventPrinter);
		pEventMgr->VRegisterHandler(Events::EventType::EXIT_TRIGGER, eventPrinter);
		pEventMgr->VRegisterHandler(Events::EventType::COLLISION_EVENT, debugPrinter);
		pEventMgr->VRegisterHandler(Events::EventType::SEPARATION_EVENT, debugPrinter);

		return true;
	}

	void DemoGameLogic::VUpdate(float deltaSec)
	{
		auto pGame = GameData::GetInstance();
		assert(pGame && pGame->GetInputStateHandler());
		m_pData->m_currentMouseState = pGame->GetInputStateHandler()->GetMouseState();

		auto pDisplay = pGame->GetDisplayComponent();
		auto pPhysics = pGame->GetPhysicsEngine();
		m_pData->m_currentCameraState =
			CameraState(pDisplay->VGetCameraPosition(), pDisplay->VGetCameraTarget());

		m_pData->UpdateHighlight(deltaSec);

		if (m_pData->RightMouseReleased())
		{
			ThrowCube(pDisplay->VGetCameraTarget());
		}

		if (m_pData->LeftMousePressed())
		{
			Vec3 pickPoint;
			ActorID pickedActorId = m_pData->PickTarget(pickPoint);

			std::cerr << "Selected actor id: ";
			if (!pickedActorId)
				std::cerr << "none";
			else
				std::cerr << pickedActorId;
			std::cerr << std::endl;

			if (pickedActorId != 0)
			{
				m_pData->m_pickConstraintId = pPhysics->VAddPickConstraint(pickedActorId, pickPoint,
					m_pData->m_currentCameraState.cameraPos);
				if (m_pData->m_pickedActor != pickedActorId)
				{
					AddHighlight(pickedActorId);
					RemoveHighlight(m_pData->m_pickedActor);
					m_pData->m_pickedActor = pickedActorId;
				}
			}
		}
		else if (m_pData->LeftMouseReleased() && m_pData->m_pickConstraintId != 0)
		{
			pPhysics->VRemoveConstraint(m_pData->m_pickedActor, m_pData->m_pickConstraintId);
			m_pData->m_pickConstraintId = 0;
		}

		if (m_pData->CameraMoved())
		{
			auto pEventMgr = pGame->GetEventManager();
			if (pEventMgr)
			{
				std::shared_ptr<Events::IEventData> event(new Events::CameraMoveEvent(pGame->CurrentTimeMs(),
					m_pData->m_currentCameraState.cameraPos, m_pData->m_currentCameraState.cameraTarget));
				pEventMgr->VQueueEvent(event);
			}
		}

		m_pData->m_previousMouseState = m_pData->m_currentMouseState;
		m_pData->m_previousCameraState = m_pData->m_currentCameraState;
	}

	bool DemoGameLogicData::CameraMoved()
	{
		return m_previousCameraState.cameraPos != m_currentCameraState.cameraPos ||
			m_previousCameraState.cameraTarget != m_currentCameraState.cameraTarget;
	}

	bool DemoGameLogicData::LeftMousePressed()
	{
		return m_currentMouseState.LeftMouseDown && !m_previousMouseState.LeftMouseDown;
	}

	bool DemoGameLogicData::LeftMouseDown()
	{
		return m_currentMouseState.LeftMouseDown && m_previousMouseState.LeftMouseDown;
	}

	bool DemoGameLogicData::LeftMouseReleased()
	{
		return !m_currentMouseState.LeftMouseDown && m_previousMouseState.LeftMouseDown;
	}

	bool DemoGameLogicData::RightMousePressed()
	{
		return m_currentMouseState.RightMouseDown && !m_previousMouseState.RightMouseDown;
	}

	bool DemoGameLogicData::RightMouseDown()
	{
		return m_currentMouseState.RightMouseDown && m_previousMouseState.RightMouseDown;
	}

	bool DemoGameLogicData::RightMouseReleased()
	{
		return !m_currentMouseState.RightMouseDown && m_previousMouseState.RightMouseDown;
	}
}