#include "CppUnitTest.h"
#include "ToString.h"
#include "MockEventReceiver.h"
#include <BulletPhysics.h>
#include <EventManager.h>
#include <GameActor.h>
#include <GameData.h>
#include <WorldTransformComponent.h>
#include <Vec3.h>
#include <Vec4.h>
#include <memory>

using namespace Microsoft::VisualStudio::CppUnitTestFramework;
using GameEngine::Physics::IPhysicsEngine;
using GameEngine::Physics::BulletPhysics;
using GameEngine::GameActor;
using GameEngine::WorldTransformComponent;
using GameEngine::Vec3;
using GameEngine::Quaternion;
using GameEngine::GameData;
using GameEngine::Events::EventManager;
using GameEngine::Events::IEventManager;

namespace
{
	const float deltaTimeStep = 1/60.f;
}

namespace DemoTest
{
	TEST_CLASS(BulletPhysicsTest)
	{
	public:
		TEST_METHOD_INITIALIZE(SetupBulletPhysics)
		{
			// TODO: make a factory for the physics engine as well,
			// to ensure initialization before use.
			pPhysics.reset(new BulletPhysics(0.05f));
			pPhysics->VInitEngine("..\\assets\\materials.xml");

			pActor.reset(new GameActor());
			pEvents.reset(new EventManager());

			auto pGame = GameData::GetInstance();
			pGame->AddActor(pActor);
			pGame->SetEventManager(pEvents);
			pGame->SetPhysicsEngine(pPhysics);
		}

		TEST_METHOD_CLEANUP(CleanupBulletPhysics)
		{
			pPhysics.reset();
			GameData::GetInstance()->RemoveActor(pActor->GetID());
			pActor.reset();
			pEvents.reset();
		}

		TEST_METHOD(DefaultGravityPointsTowardNegativeYAxis)
		{
			Vec3 actorStartPosition(0, 100, 0);
			pActor->GetWorldTransform().SetPosition(actorStartPosition);
			pPhysics->VAddSphere(pActor, 10.f,
				GameEngine::Physics::IPhysicsEngine::PhysicsObjectType::DYNAMIC,
				"balsa", "Normal");

			pPhysics->VUpdateSimulation(deltaTimeStep);
			pPhysics->VSyncScene();

			Vec3 actorPosition = pActor->GetWorldTransform().GetPosition();
			Vec3 gravityVector = (actorPosition - actorStartPosition).normalized();
			Assert::AreEqual(Vec3(0, -1, 0), gravityVector);
		}

		// TODO: Test applying a linear force on an object.
		// TODO: Test applying an angular force on an object.
		// TODO: Test applying an impulse (force and torque).
		// TODO: Test removing a physics object.
		// TODO: Test effects of different materials?
		// TODO: Test collision events.
		// TODO: Test separation events.
		// TODO: Test trigger entry events.
		// TODO: Test trigger exit events.
		// TODO: Test stopping an actor.
		// TODO: Test raycasts:
		// - subtask: trying to select a trigger
		// - subtask: trying to select a static object
		// - subtask: trying to select a dynamic object behind a static object
		// - subtask: trying to select a dynamic object behind a trigger
		// - subtask: trying to select a dynamic object inside a trigger
		//   (from outside the trigger)
		// TODO: Test adding a pick constraint and sending camera move events.
		// - subtask: refactor pick constraints
		// TODO: Test removing a pick constraint and sending camera move events.
	private:
		std::shared_ptr<IPhysicsEngine> pPhysics;
		std::shared_ptr<IEventManager> pEvents;
		std::shared_ptr<GameActor> pActor;
	};
}