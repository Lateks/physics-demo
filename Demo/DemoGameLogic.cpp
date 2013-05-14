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

	int NUM_CUBES = 0;

	const float HL_UPDATE_INTERVAL = 1.f/60;
	const float PI = 3.14159f;
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
		float m_pickDistance;
		Vec3 m_storedAngularFactor;

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
		void AddPickConstraint(ActorID actorId, const Vec3& pickPoint, const Vec3& cameraPos);
		void UpdatePickConstraint();
		void RemovePickConstraint();
	};

	// Highlights the given actor's scene node with a blue hued light.
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

	void DemoGameLogicData::AddPickConstraint(ActorID actorId, const Vec3& pickPoint, const Vec3& cameraPos)
	{
		auto pPhysics = GameData::GetInstance()->GetPhysicsEngine();
		if (pPhysics)
		{
			m_pickConstraintId = pPhysics->VAddDOF6Constraint(actorId, pickPoint);
			m_pickDistance = (pickPoint - cameraPos).norm();

			// Disable rotations to avoid jitter when rigid body is
			// pushed against e.g. a wall. Store the current angular
			// factor to restore it later. Set angular velocity to zero
			// to stop ongoing rotations at the moment of picking.
			m_storedAngularFactor = pPhysics->VGetAngularFactor(actorId);
			pPhysics->VSetAngularFactor(actorId, Vec3(0.f, 0.f, 0.f));
			pPhysics->VSetAngularVelocity(actorId, Vec3(1, 0, 0), 0.f);

			// Update highlights if required.
			if (m_pickedActor != actorId)
			{
				AddHighlight(actorId);
				RemoveHighlight(m_pickedActor);
				m_pickedActor = actorId;
			}
		}
	}

	void DemoGameLogicData::UpdatePickConstraint()
	{
		auto pPhysics = GameData::GetInstance()->GetPhysicsEngine();
		// Update the pivot to the current camera target but try to keep
		// the object at the original picking distance.
		if (pPhysics && m_pickConstraintId != 0)
		{
			Vec3 cameraDirection = (m_currentCameraState.cameraTarget - m_currentCameraState.cameraPos).normalized();
			Vec3 newPivot = m_currentCameraState.cameraPos + cameraDirection * m_pickDistance;
			pPhysics->VUpdateDOF6PivotPoint(m_pickConstraintId, newPivot);
		}
	}

	void DemoGameLogicData::RemovePickConstraint()
	{
		auto pPhysics = GameData::GetInstance()->GetPhysicsEngine();
		if (pPhysics && m_pickConstraintId != 0)
		{
			pPhysics->VRemoveConstraint(m_pickConstraintId);
			pPhysics->VSetAngularFactor(m_pickedActor, m_storedAngularFactor);
			m_pickConstraintId = 0;
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
			Vec3 targetVector = (m_currentCameraState.cameraTarget - rayFrom).normalized() * 500.f;
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

	void AppendActorNameForDisplay(std::wostream& stream, ActorID id)
	{
		auto pActor = GameData::GetInstance()->GetActor(id);
		if (!pActor)
			stream << L"unidentified actor";
		else if (pActor->GetName().empty())
			stream << L"actor #" << pActor->GetID();
		else
			stream << pActor->GetName();
	}

	void PrintTriggerEvent(Events::EventPtr event)
	{
		auto pDisplay = GameData::GetInstance()->GetDisplayComponent();
		if (!pDisplay) return;

		auto pMessages = pDisplay->VGetMessageWindow();
		if (pMessages)
		{
			std::shared_ptr<Events::TriggerEvent> pEvent =
				std::dynamic_pointer_cast<Events::TriggerEvent>(event);
			std::wstringstream message;
			if (event->VGetEventType() == Events::EventType::ENTER_TRIGGER)
			{
				message << L"ENTERED";
			}
			else if (event->VGetEventType() == Events::EventType::EXIT_TRIGGER)
			{
				message << L"EXITED";
			}
			message << " TRIGGER: ";
			AppendActorNameForDisplay(message, pEvent->GetActorId());
			pMessages->VAddMessage(message.str());
		}
	}

	void PrintCollisionEvent(Events::EventPtr pEvent)
	{
		bool isCollisionEvent = pEvent->VGetEventType() == Events::EventType::COLLISION_EVENT ||
			pEvent->VGetEventType() == Events::EventType::SEPARATION_EVENT;
		assert(isCollisionEvent);
		if (!isCollisionEvent)
			return;

		auto pDisplay = GameData::GetInstance()->GetDisplayComponent();
		if (!pDisplay) return;

		auto pMessages = pDisplay->VGetMessageWindow();
		if (pMessages)
		{
			std::wstringstream message;
			auto pCollisionEvent = std::dynamic_pointer_cast<Events::CollisionEvent>(pEvent);
			if (pEvent->VGetEventType() == Events::EventType::COLLISION_EVENT)
				message << L"COLLIDED: ";
			else
				message << L"SEPARATED: ";
			auto collisionPair = pCollisionEvent->GetCollisionPair();
			AppendActorNameForDisplay(message, collisionPair.first);
			message << L" and ";
			AppendActorNameForDisplay(message, collisionPair.second);

			pMessages->VAddMessage(message.str());
		}
	}

	void ThrowCube(Vec3& throwTowards)
	{
		++NUM_CUBES;
		auto pGame = GameData::GetInstance();
		auto pDisplay = pGame->GetDisplayComponent();
		Vec3 cameraPos = pDisplay->VGetCameraPosition();

		std::wstringstream actorName;
		actorName << L"small box #" << NUM_CUBES;
		auto cube = std::make_shared<GameActor>(cameraPos, actorName.str());
		cube->GetWorldTransform().SetRotation(pDisplay->VGetCameraRotation());
		pGame->AddActor(cube);

		pDisplay->VAddCubeSceneNode(15.f, cube, WOODBOX_TEXTURE, true);
		auto physics = pGame->GetPhysicsEngine();
		physics->VAddBox(cube, Vec3(15.f, 15.f, 15.f),
			Physics::IPhysicsEngine::PhysicsObjectType::DYNAMIC, "Titanium", "Bouncy");

		Vec3 throwDirection = throwTowards - cameraPos;

		// Make the object rotate slightly away from the camera.
		Vec3 rotationAxis = -1.f * pDisplay->VGetCameraRightVector();

		physics->VSetLinearVelocity(cube->GetID(), throwDirection, 175.f);
		physics->VSetAngularVelocity(cube->GetID(), rotationAxis, PI);
	}

	DemoGameLogic::DemoGameLogic() : m_pData(new DemoGameLogicData())
	{
		m_pData->m_updateTimer = 0.f;
		m_pData->m_pickConstraintId = 0;
		m_pData->m_pickedActor = 0;
	}

	DemoGameLogic::~DemoGameLogic() { }

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

		pDisplay->VSetCameraFOV(75.f);
		pDisplay->VHideCursor();

		pMessages->VSetFont("..\\assets\\fontcourier.bmp");
		pMessages->VSetVisible(true);
		pMessages->VSetWidth(600);

		// Create an actor for the world map to be able to refer to the associated
		// rigid bodies.
		Vec3 mapPosition(-1350, -130, -1400);
		auto world = std::make_shared<GameActor>(mapPosition, L"worldmap");
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
		auto ball = std::make_shared<GameActor>(Vec3(0, 50, -60), L"stone ball");
		pGame->AddActor(ball);
		pDisplay->VAddSphereSceneNode(10.f, ball, MUD_TEXTURE, true);
		pPhysics->VAddSphere(ball, 10.f, Physics::IPhysicsEngine::PhysicsObjectType::DYNAMIC, "Titanium", "Bouncy");

		auto cube = std::make_shared<GameActor>(Vec3(0, 80, -60), L"large box");
		pGame->AddActor(cube);
		pDisplay->VAddCubeSceneNode(25.f, cube, WOODBOX_TEXTURE, true);
		pPhysics->VAddBox(cube, Vec3(25.f, 25.f, 25.f), Physics::IPhysicsEngine::PhysicsObjectType::DYNAMIC, "manganese", "Normal");

		// Add a trigger node, rendered as a wireframe cube.
		auto trigger = std::make_shared<GameActor>(Vec3(-100, 125, -450), L"trigger cube");
		pGame->AddActor(trigger);
		pDisplay->VAddCubeSceneNode(125.f, trigger, 0);
		pPhysics->VAddBox(trigger, Vec3(125.f, 125.f, 125.f), Physics::IPhysicsEngine::PhysicsObjectType::TRIGGER);

		// Create event handler to print out messages to the in-game message window
		// when a trigger event is detected.
		Events::EventHandlerPtr eventPrinter(new std::function<void(Events::EventPtr)>
			([] (Events::EventPtr event)
		{
			PrintTriggerEvent(event);
		}));

		// Create event handler to print collision events to the console as debug information.
		Events::EventHandlerPtr debugPrinter(new std::function<void(Events::EventPtr)>
			([] (Events::EventPtr event)
		{
			PrintCollisionEvent(event);
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

			std::wcerr << L"Selected actor: ";
			if (!pickedActorId)
				std::wcerr << L"none";
			else
				AppendActorNameForDisplay(std::wcerr, pickedActorId);
			std::wcerr << std::endl;

			if (pickedActorId != 0)
			{
				m_pData->AddPickConstraint(pickedActorId, pickPoint,
					m_pData->m_currentCameraState.cameraPos);
			}
		}
		else if (m_pData->LeftMouseReleased() && m_pData->m_pickConstraintId != 0)
		{
			m_pData->RemovePickConstraint();
		}

		if (m_pData->CameraMoved())
		{
			m_pData->UpdatePickConstraint();
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