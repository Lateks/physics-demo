#include "CppUnitTest.h"
#include "ToString.h"
#include "MockEventReceiver.h"
#include "EqualityComparison.h"
#include <BulletPhysics.h>
#include <EventManager.h>
#include <GameActor.h>
#include <GameData.h>
#include <WorldTransformComponent.h>
#include <Vec3.h>
#include <Vec4.h>
#include <IrrlichtConversions.h> // used for conversions between quaternions and euler angles
#include <memory>

using namespace Microsoft::VisualStudio::CppUnitTestFramework;
using GameEngine::Physics::IPhysicsEngine;
using GameEngine::Physics::BulletPhysics;
using GameEngine::Events::EventType;
using GameEngine::GameActor;
using GameEngine::WorldTransformComponent;
using GameEngine::Vec3;
using GameEngine::Quaternion;
using GameEngine::GameData;
using GameEngine::Events::EventManager;
using GameEngine::Events::IEventManager;

namespace
{
	const float DELTA_TIME_STEP = 1/60.f;
	const float PI = 3.14159f;
	const float WORLD_SCALE = 0.05f;
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
			pPhysics.reset(new BulletPhysics(WORLD_SCALE));
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
				IPhysicsEngine::PhysicsObjectType::DYNAMIC, "balsa", "Normal");

			pPhysics->VUpdateSimulation(DELTA_TIME_STEP);
			pPhysics->VSyncScene();

			Vec3 actorPosition = pActor->GetWorldTransform().GetPosition();
			Vec3 gravityVector = (actorPosition - actorStartPosition).normalized();
			Assert::AreEqual(Vec3(0, -1, 0), gravityVector);
		}

		TEST_METHOD(DefaultGravityAssumesSingleUnitToBeCloseToAMeter)
		{
			Vec3 gravity = pPhysics->VGetGlobalGravity();
			Assert::AreEqual(10.f/WORLD_SCALE, gravity.norm());
		}

		TEST_METHOD(ApplyingAForce)
		{
			Vec3 actorStartPosition(0, 100, 0);
			pActor->GetWorldTransform().SetPosition(actorStartPosition);
			pPhysics->VAddSphere(pActor, 10.f,
				IPhysicsEngine::PhysicsObjectType::DYNAMIC, "balsa", "Normal");

			Vec3 directionOfForce(1, 0, 0);
			pPhysics->VApplyForce(directionOfForce, 10.f, pActor->GetID());
			pPhysics->VSetGlobalGravity(Vec3(0, 0, 0)); // disregard gravity

			pPhysics->VUpdateSimulation(DELTA_TIME_STEP);
			pPhysics->VSyncScene();

			Vec3 actorPosition = pActor->GetWorldTransform().GetPosition();
			Vec3 movementVector = (actorPosition - actorStartPosition).normalized();
			Assert::AreEqual(directionOfForce, movementVector);
		}

		TEST_METHOD(SettingLinearVelocity)
		{
			Vec3 actorStartPosition(0, 100, 0);
			pActor->GetWorldTransform().SetPosition(actorStartPosition);
			pPhysics->VAddSphere(pActor, 10.f,
				IPhysicsEngine::PhysicsObjectType::DYNAMIC, "balsa", "Normal");

			Vec3 directionOfVelocity(1, 0, 0);
			pPhysics->VSetLinearVelocity(pActor->GetID(), directionOfVelocity, 10.f);
			pPhysics->VSetGlobalGravity(Vec3(0, 0, 0)); // disregard gravity

			pPhysics->VUpdateSimulation(DELTA_TIME_STEP);
			pPhysics->VSyncScene();

			Vec3 actorPosition = pActor->GetWorldTransform().GetPosition();
			Vec3 movementVector = (actorPosition - actorStartPosition).normalized();
			Assert::AreEqual(directionOfVelocity, movementVector);
		}

		TEST_METHOD(SettingAngularVelocity)
		{
			Vec3 actorStartPosition(0, 100, 0);
			pActor->GetWorldTransform().SetPosition(actorStartPosition);
			Vec3 startRotation = GameEngine::ConvertVector(
				GameEngine::QuaternionToEuler(pActor->GetWorldTransform().GetRotation()));
			pPhysics->VAddSphere(pActor, 10.f,
				IPhysicsEngine::PhysicsObjectType::DYNAMIC, "balsa", "Normal");

			// Set an angular velocity about the x axis in radians per second.
			pPhysics->VSetAngularVelocity(pActor->GetID(), Vec3(1, 0, 0), PI);
			pPhysics->VSetGlobalGravity(Vec3(0, 0, 0)); // disregard gravity

			pPhysics->VUpdateSimulation(DELTA_TIME_STEP);
			pPhysics->VSyncScene();

			Vec3 actorRotation = GameEngine::ConvertVector(
				GameEngine::QuaternionToEuler(pActor->GetWorldTransform().GetRotation()));
			Vec3 difference = actorRotation-startRotation;
			Assert::AreEqual(0.f, difference.y());
			Assert::AreEqual(0.f, difference.z());
			Assert::IsTrue(AreEqual(PI*DELTA_TIME_STEP, difference.x(), 0.00001f));
		}

		TEST_METHOD(ApplyingATorque)
		{
			Vec3 actorStartPosition(0, 100, 0);
			pActor->GetWorldTransform().SetPosition(actorStartPosition);
			Vec3 startRotation = GameEngine::ConvertVector(
				GameEngine::QuaternionToEuler(pActor->GetWorldTransform().GetRotation()));
			pPhysics->VAddSphere(pActor, 10.f,
				GameEngine::Physics::IPhysicsEngine::PhysicsObjectType::DYNAMIC,
				"balsa", "Normal");

			pPhysics->VApplyTorque(Vec3(1, 0, 0), PI, pActor->GetID());
			pPhysics->VSetGlobalGravity(Vec3(0, 0, 0)); // disregard gravity

			pPhysics->VUpdateSimulation(DELTA_TIME_STEP);
			pPhysics->VSyncScene();

			Vec3 actorRotation = GameEngine::ConvertVector(
				GameEngine::QuaternionToEuler(pActor->GetWorldTransform().GetRotation()));
			Vec3 difference = actorRotation - startRotation;
			Assert::AreEqual(0.f, difference.y());
			Assert::AreEqual(0.f, difference.z());
			Assert::IsTrue(difference.x() > 0.f);
		}

		TEST_METHOD(StopAMovingActor)
		{
			Vec3 actorStartPosition(0, 100, 0);
			pActor->GetWorldTransform().SetPosition(actorStartPosition);
			pPhysics->VAddSphere(pActor, 10.f,
				GameEngine::Physics::IPhysicsEngine::PhysicsObjectType::DYNAMIC,
				"balsa", "Normal");

			pPhysics->VSetLinearVelocity(pActor->GetID(), Vec3(1, 0, 0), 10.f);
			pPhysics->VSetAngularVelocity(pActor->GetID(), Vec3(-1, 0, 0), PI);
			pPhysics->VSetGlobalGravity(Vec3(0, 0, 0)); // disregard gravity

			pPhysics->VUpdateSimulation(DELTA_TIME_STEP);
			pPhysics->VSyncScene();
			Vec3 oldActorPosition = pActor->GetWorldTransform().GetPosition();
			Quaternion oldActorRotation = pActor->GetWorldTransform().GetRotation();

			pPhysics->VStopActor(pActor->GetID());
			pPhysics->VUpdateSimulation(DELTA_TIME_STEP);
			pPhysics->VSyncScene();

			Assert::AreEqual(oldActorPosition, pActor->GetWorldTransform().GetPosition());
			Assert::IsTrue(AreEqual(oldActorRotation, pActor->GetWorldTransform().GetRotation(), 0.00001f));
		}

		TEST_METHOD(RemovingAPhysicsWorldObject)
		{
			Vec3 actorStartPosition(0, 100, 0);
			pActor->GetWorldTransform().SetPosition(actorStartPosition);
			pPhysics->VAddSphere(pActor, 10.f,
				GameEngine::Physics::IPhysicsEngine::PhysicsObjectType::DYNAMIC,
				"balsa", "Normal");

			pPhysics->VSetLinearVelocity(pActor->GetID(), Vec3(1, 0, 0), 10.f);
			pPhysics->VSetAngularVelocity(pActor->GetID(), Vec3(-1, 0, 0), PI);

			pPhysics->VUpdateSimulation(DELTA_TIME_STEP);
			pPhysics->VSyncScene();
			Vec3 oldActorPosition = pActor->GetWorldTransform().GetPosition();
			Quaternion oldActorRotation = pActor->GetWorldTransform().GetRotation();

			pPhysics->VRemoveActor(pActor->GetID()); // should no longer move or be affected by gravity after removal
			pPhysics->VUpdateSimulation(DELTA_TIME_STEP);
			pPhysics->VSyncScene();

			Assert::AreEqual(oldActorPosition, pActor->GetWorldTransform().GetPosition());
			Assert::IsTrue(AreEqual(oldActorRotation, pActor->GetWorldTransform().GetRotation(), 0.00001f));
		}

		TEST_METHOD(CollisionEventIsSentWhenObjectsCloseEnough)
		{
			auto pGame = GameData::GetInstance();
			Vec3 actorStartPosition(0, 100, 0);
			pActor->GetWorldTransform().SetPosition(actorStartPosition);
			pPhysics->VAddSphere(pActor, 10.f, IPhysicsEngine::PhysicsObjectType::DYNAMIC,
				"balsa", "Normal");

			// Add another object to collide with.
			auto pActor2 = std::make_shared<GameActor>();
			pGame->AddActor(pActor);
			Vec3 distance(75.f, 0, 0);
			Vec3 actor2StartPosition(actorStartPosition + distance);
			pActor2->GetWorldTransform().SetPosition(actor2StartPosition);
			pPhysics->VAddSphere(pActor2, 50.f, IPhysicsEngine::PhysicsObjectType::DYNAMIC);

			pPhysics->VSetGlobalGravity(Vec3(0, 0, 0));

			/* Move 20 units/sec toward the other object. Because the distance
			 * between the objects outer perimeters is 20 units, the objects
			 * should collide within a second. In practice, Bullet detects
			 * this collision slightly earlier (probably an issue with the
			 * contact processing threshold?).
			 */
			pPhysics->VSetLinearVelocity(pActor->GetID(), distance, 20.f);

			MockEventReceiver receiver;
			receiver.RegisterTo(EventType::COLLISION_EVENT, pGame->GetEventManager());

			for (int i = 0; i < 60; ++i)
			{
				pPhysics->VUpdateSimulation(DELTA_TIME_STEP);
				pPhysics->VSyncScene();
				pGame->GetEventManager()->VDispatchEvents();
			}
			Assert::AreEqual(1, receiver.NumValidCallsReceived());
		}

		TEST_METHOD(SeparationEventSentWhenObjectsFarEnough)
		{
			auto pGame = GameData::GetInstance();
			Vec3 actorStartPosition(0, 100, 0);
			pActor->GetWorldTransform().SetPosition(actorStartPosition);
			pPhysics->VAddSphere(pActor, 10.f,
				IPhysicsEngine::PhysicsObjectType::DYNAMIC, "balsa", "Normal");

			// Add another object to separate from.
			auto pActor2 = std::make_shared<GameActor>();
			pGame->AddActor(pActor);
			Vec3 distance(50.f, 0, 0);
			Vec3 actor2StartPosition(actorStartPosition + distance);
			pActor2->GetWorldTransform().SetPosition(actor2StartPosition);
			pPhysics->VAddSphere(pActor2, 40.f, IPhysicsEngine::PhysicsObjectType::DYNAMIC);

			pPhysics->VSetGlobalGravity(Vec3(0, 0, 0));
			pPhysics->VSetLinearVelocity(pActor->GetID(), Vec3(-1.f * distance), 20.f);

			MockEventReceiver receiver;
			receiver.RegisterTo(EventType::SEPARATION_EVENT, pGame->GetEventManager());

			for (int i = 0; i < 30; ++i)
			{
				pPhysics->VUpdateSimulation(DELTA_TIME_STEP);
				pPhysics->VSyncScene();
				pGame->GetEventManager()->VDispatchEvents();
			}
			Assert::AreEqual(1, receiver.NumValidCallsReceived());
		}

		// TODO: Test trigger entry events.
		// TODO: Test trigger exit events.
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
		// TODO: Test removal of a physics world object when it is affected by a constraint
		// TODO: Additional tests for gravity
		// TODO: Test effects of different materials?
	private:
		std::shared_ptr<IPhysicsEngine> pPhysics;
		std::shared_ptr<IEventManager> pEvents;
		std::shared_ptr<GameActor> pActor;
	};
}