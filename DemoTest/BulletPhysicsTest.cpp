#include "CppUnitTest.h"
#include "ToString.h"
#include "MockEventReceiver.h"
#include "EqualityComparison.h"
#include "Events.h"
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
	const GameEngine::ActorID NO_ACTOR = 0u;
}

namespace DemoTest
{
	TEST_CLASS(BulletPhysicsTest)
	{
	public:
		TEST_METHOD_INITIALIZE(SetupBulletPhysics)
		{
			GameEngine::Physics::BulletPhysicsFactory factory("..\\assets\\materials.xml", WORLD_SCALE);
			pPhysics = factory.CreatePhysicsEngine();

			pActor.reset(new GameActor());
			pEvents.reset(new EventManager());

			auto pGame = GameData::GetInstance();
			pGame->AddActor(pActor);
			pGame->SetEventManager(pEvents);
			pGame->SetPhysicsEngine(pPhysics);
		}

		TEST_METHOD_CLEANUP(CleanupBulletPhysics)
		{
			auto pGame = GameData::GetInstance();
			pGame->SetPhysicsEngine(nullptr);
			pPhysics.reset();
			pGame->RemoveActor(pActor->GetID());
			pActor.reset();
			pGame->SetEventManager(nullptr);
			pEvents.reset();
		}

		TEST_METHOD(SystemCannotBeReinitialized)
		{
			Assert::IsFalse(pPhysics->VInitEngine("..\\assets\\materials.xml"));
		}

		TEST_METHOD(SystemDoesNotCrashWhenNullActorPointerGiven)
		{
			pPhysics->VAddSphere(nullptr, 10.f,
				IPhysicsEngine::PhysicsObjectType::DYNAMIC);
			pPhysics->VAddBox(nullptr, Vec3(10.f, 10.f, 10.f),
				IPhysicsEngine::PhysicsObjectType::DYNAMIC);
			pPhysics->VAddConvexMesh(nullptr, std::vector<GameEngine::Vec4>(),
				IPhysicsEngine::PhysicsObjectType::DYNAMIC);
			pPhysics->VAddConvexMesh(nullptr, std::vector<Vec3>(),
				IPhysicsEngine::PhysicsObjectType::DYNAMIC);

			pPhysics->VUpdateSimulation(DELTA_TIME_STEP);
			pPhysics->VSyncScene();
		}

		TEST_METHOD(GravityAffectsObjectsWithNonZeroMass)
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

		TEST_METHOD(GravityDoesNotAffectObjectsWithNoMass)
		{
			Vec3 actorStartPosition(0, 100, 0);
			pActor->GetWorldTransform().SetPosition(actorStartPosition);
			pPhysics->VAddSphere(pActor, 10.f,
				IPhysicsEngine::PhysicsObjectType::DYNAMIC); // no specified material = no mass

			pPhysics->VUpdateSimulation(DELTA_TIME_STEP);
			pPhysics->VSyncScene();

			Assert::AreEqual(actorStartPosition, pActor->GetWorldTransform().GetPosition());
		}

		TEST_METHOD(GravityDoesNotAffectStaticObjects)
		{
			Vec3 actorStartPosition(0, 100, 0);
			pActor->GetWorldTransform().SetPosition(actorStartPosition);
			pPhysics->VAddSphere(pActor, 10.f,
				IPhysicsEngine::PhysicsObjectType::STATIC, "Titanium", "");

			pPhysics->VUpdateSimulation(DELTA_TIME_STEP);
			pPhysics->VSyncScene();

			Assert::AreEqual(actorStartPosition, pActor->GetWorldTransform().GetPosition());
		}

		TEST_METHOD(ApplyingAForce)
		{
			Vec3 actorStartPosition(0, 100, 0);
			pActor->GetWorldTransform().SetPosition(actorStartPosition);
			pPhysics->VAddSphere(pActor, 10.f,
				IPhysicsEngine::PhysicsObjectType::DYNAMIC, "balsa", "Normal");

			Vec3 directionOfForce(1, 0, 0);
			pPhysics->VApplyForce(pActor->GetID(), directionOfForce, 10.f);
			pPhysics->VSetGlobalGravity(Vec3(0, 0, 0)); // disregard gravity

			pPhysics->VUpdateSimulation(DELTA_TIME_STEP);
			pPhysics->VSyncScene();

			Vec3 actorPosition = pActor->GetWorldTransform().GetPosition();
			Vec3 movementVector = (actorPosition - actorStartPosition).normalized();
			Assert::AreEqual(directionOfForce, movementVector);
		}

		TEST_METHOD(ForcesCannotBeAppliedToObjectsWithoutMass)
		{
			Vec3 actorStartPosition(0, 100, 0);
			pActor->GetWorldTransform().SetPosition(actorStartPosition);
			pPhysics->VAddSphere(pActor, 10.f,
				IPhysicsEngine::PhysicsObjectType::DYNAMIC);

			Vec3 directionOfForce(1, 0, 0);
			pPhysics->VApplyForce(pActor->GetID(), directionOfForce, 10.f);

			pPhysics->VUpdateSimulation(DELTA_TIME_STEP);
			pPhysics->VSyncScene();

			Vec3 actorPosition = pActor->GetWorldTransform().GetPosition();
			float movementVectorLength = (actorPosition - actorStartPosition).norm();
			Assert::AreEqual(0.f, movementVectorLength);
		}

		TEST_METHOD(ForcesCannotBeAppliedToStaticObjects)
		{
			Vec3 actorStartPosition(0, 100, 0);
			pActor->GetWorldTransform().SetPosition(actorStartPosition);
			pPhysics->VAddSphere(pActor, 10.f,
				IPhysicsEngine::PhysicsObjectType::STATIC, "balsa", "Normal");

			Vec3 directionOfForce(1, 0, 0);
			pPhysics->VApplyForce(pActor->GetID(), directionOfForce, 10.f);

			pPhysics->VUpdateSimulation(DELTA_TIME_STEP);
			pPhysics->VSyncScene();

			Vec3 actorPosition = pActor->GetWorldTransform().GetPosition();
			float movementVectorLength = (actorPosition - actorStartPosition).norm();
			Assert::AreEqual(0.f, movementVectorLength);
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

			// Set an angular velocity about the negative x axis in radians per second.
			pPhysics->VSetAngularVelocity(pActor->GetID(), Vec3(-1, 0, 0), PI);
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

		// Check that radians per second are not inconsistently scaled as return values
		// (they should never be scaled).
		TEST_METHOD(RadiansPerSecondAreNotScaled)
		{
			Vec3 actorStartPosition(0, 100, 0);
			pActor->GetWorldTransform().SetPosition(actorStartPosition);
			Vec3 startRotation = GameEngine::ConvertVector(
				GameEngine::QuaternionToEuler(pActor->GetWorldTransform().GetRotation()));
			pPhysics->VAddSphere(pActor, 10.f,
				IPhysicsEngine::PhysicsObjectType::DYNAMIC, "balsa", "Normal");

			// Set an angular velocity about the negative x axis in radians per second.
			Vec3 dir(-1, 0, 0);
			pPhysics->VSetAngularVelocity(pActor->GetID(), dir, PI);
			Assert::AreEqual(Vec3(PI * dir), pPhysics->VGetAngularVelocity(pActor->GetID()));
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

			pPhysics->VApplyTorque(pActor->GetID(), Vec3(-1, 0, 0), PI);
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

		TEST_METHOD(StoppingAMovingActor)
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
			 * between the objects' outer perimeters is 20 units, the objects
			 * should collide within about a second.
			 */
			pPhysics->VSetLinearVelocity(pActor->GetID(), distance, 20.f);

			MockEventReceiver receiver;
			receiver.RegisterTo(EventType::COLLISION_EVENT, pGame->GetEventManager());

			SimulateSteps(45);
			Assert::AreEqual(0, receiver.NumCallsReceived());
			SimulateSteps(15);
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

			SimulateSteps(30);
			Assert::AreEqual(1, receiver.NumValidCallsReceived());
		}

		TEST_METHOD(TriggerEntryEventIsSentWhenObjectIsCloseEnoughToTrigger)
		{
			auto pGame = GameData::GetInstance();
			Vec3 actorStartPosition(0, 100, 0);
			pActor->GetWorldTransform().SetPosition(actorStartPosition);
			pPhysics->VAddSphere(pActor, 10.f, IPhysicsEngine::PhysicsObjectType::DYNAMIC,
				"balsa", "Normal");

			auto pActor2 = std::make_shared<GameActor>();
			pGame->AddActor(pActor);
			Vec3 distance(75.f, 0, 0);
			Vec3 actor2StartPosition(actorStartPosition + distance);
			pActor2->GetWorldTransform().SetPosition(actor2StartPosition);
			pPhysics->VAddBox(pActor2, Vec3(100.f, 100.f, 100.f),
				IPhysicsEngine::PhysicsObjectType::TRIGGER);

			pPhysics->VSetGlobalGravity(Vec3(0, 0, 0));
			pPhysics->VSetLinearVelocity(pActor->GetID(), distance, 20.f);

			MockEventReceiver receiver;
			receiver.RegisterTo(EventType::ENTER_TRIGGER, pGame->GetEventManager());

			SimulateSteps(30);
			Assert::AreEqual(0, receiver.NumCallsReceived());
			SimulateSteps(30);
			Assert::AreEqual(1, receiver.NumValidCallsReceived());
		}

		TEST_METHOD(TriggerExitEventIsSentWhenObjectMovesFarEnoughFromTrigger)
		{
			auto pGame = GameData::GetInstance();
			Vec3 actorStartPosition(0, 100, 0);
			pActor->GetWorldTransform().SetPosition(actorStartPosition);
			pPhysics->VAddSphere(pActor, 10.f,
				IPhysicsEngine::PhysicsObjectType::DYNAMIC, "balsa", "Normal");

			auto pActor2 = std::make_shared<GameActor>();
			pGame->AddActor(pActor);
			Vec3 distance(40.f, 0, 0);
			Vec3 actor2StartPosition(actorStartPosition + distance);
			pActor2->GetWorldTransform().SetPosition(actor2StartPosition);
			pPhysics->VAddBox(pActor2,
				Vec3(80.f, 80.f, 80.f), IPhysicsEngine::PhysicsObjectType::TRIGGER);

			pPhysics->VSetGlobalGravity(Vec3(0, 0, 0));
			pPhysics->VSetLinearVelocity(pActor->GetID(), Vec3(-1.f * distance), 30.f);

			MockEventReceiver receiver;
			receiver.RegisterTo(EventType::EXIT_TRIGGER, pGame->GetEventManager());

			SimulateSteps(30);
			Assert::AreEqual(1, receiver.NumValidCallsReceived());
		}

		TEST_METHOD(RayTestReturnsZeroIfNoObjectInRange)
		{
			Vec3 actorStartPosition(0, 100, 100);
			pActor->GetWorldTransform().SetPosition(actorStartPosition);
			pPhysics->VAddSphere(pActor, 10.f,
				IPhysicsEngine::PhysicsObjectType::DYNAMIC, "balsa", "Normal");

			Vec3 pickPosition;
			GameEngine::ActorID id = pPhysics->VGetClosestActorHit(
				Vec3(10, 10, -10), Vec3(12, 90, 100), pickPosition);
			Assert::AreEqual(NO_ACTOR, id);
			Assert::AreEqual(Vec3(), pickPosition);
		}

		TEST_METHOD(RayTestReturnsObjectIdIfDynamicObjectSelected)
		{
			Vec3 actorStartPosition(0, 100, 100);
			pActor->GetWorldTransform().SetPosition(actorStartPosition);
			float sphereRadius = 10.f;
			pPhysics->VAddSphere(pActor, sphereRadius,
				IPhysicsEngine::PhysicsObjectType::DYNAMIC, "balsa", "Normal");

			Vec3 pickPosition;
			GameEngine::ActorID id = pPhysics->VGetClosestActorHit(
				Vec3(10, 10, -10), Vec3(5, 98, 95), pickPosition);
			Assert::AreEqual(pActor->GetID(), id);
			float distance = (actorStartPosition - pickPosition).norm();
			Assert::IsTrue(AreEqual(sphereRadius, distance, 0.04f));
		}

		TEST_METHOD(RayTestCannotPickATrigger)
		{
			Vec3 actorStartPosition(0, 50, 100);
			pActor->GetWorldTransform().SetPosition(actorStartPosition);
			pPhysics->VAddSphere(pActor, 40.f,
				IPhysicsEngine::PhysicsObjectType::TRIGGER);

			Vec3 pickPosition;
			GameEngine::ActorID id = pPhysics->VGetClosestActorHit(
				Vec3(0, 0, 0), actorStartPosition, pickPosition);
			Assert::AreEqual(NO_ACTOR, id);
			Assert::AreEqual(Vec3(), pickPosition);
		}

		TEST_METHOD(RayTestCannotPickAStaticObject)
		{
			Vec3 actorStartPosition(0, 50, 100);
			pActor->GetWorldTransform().SetPosition(actorStartPosition);
			pPhysics->VAddSphere(pActor, 40.f,
				IPhysicsEngine::PhysicsObjectType::STATIC, "", "Normal");

			Vec3 pickPosition;
			GameEngine::ActorID id = pPhysics->VGetClosestActorHit(
				Vec3(0, 0, 0), actorStartPosition, pickPosition);
			Assert::AreEqual(NO_ACTOR, id);
			Assert::AreEqual(Vec3(), pickPosition);
		}

		TEST_METHOD(RayTestCannotPickADynamicObjectBehindAStaticObject)
		{
			Vec3 actorStartPosition(100, 50, 0);
			pActor->GetWorldTransform().SetPosition(actorStartPosition);
			pPhysics->VAddBox(pActor, Vec3(40.f, 40.f, 40.f),
				IPhysicsEngine::PhysicsObjectType::STATIC, "", "Normal");

			auto pGame = GameData::GetInstance();
			auto pActor2 = std::make_shared<GameActor>();
			pGame->AddActor(pActor2);
			Vec3 actor2StartPosition(actorStartPosition + Vec3(50, 0, 0));
			pActor2->GetWorldTransform().SetPosition(actor2StartPosition);
			pPhysics->VAddSphere(pActor2, 10.f,
				IPhysicsEngine::PhysicsObjectType::DYNAMIC, "balsa", "Normal");

			Vec3 pickPosition;
			GameEngine::ActorID id = pPhysics->VGetClosestActorHit(
				Vec3(0, 50, 0), actor2StartPosition, pickPosition);
			Assert::AreEqual(NO_ACTOR, id);
			Assert::AreEqual(Vec3(), pickPosition);
		}

		TEST_METHOD(RayTestCanPickADynamicObjectBehindATrigger)
		{
			Vec3 actorStartPosition(100, 50, 0);
			pActor->GetWorldTransform().SetPosition(actorStartPosition);
			pPhysics->VAddBox(pActor, Vec3(40.f, 40.f, 40.f),
				IPhysicsEngine::PhysicsObjectType::TRIGGER);

			auto pGame = GameData::GetInstance();
			auto pActor2 = std::make_shared<GameActor>();
			pGame->AddActor(pActor2);
			Vec3 actor2StartPosition(actorStartPosition + Vec3(50, 0, 0));
			pActor2->GetWorldTransform().SetPosition(actor2StartPosition);

			float sphereRadius = 10.f;
			pPhysics->VAddSphere(pActor2, sphereRadius,
				IPhysicsEngine::PhysicsObjectType::DYNAMIC, "balsa", "Normal");

			Vec3 pickPosition;
			GameEngine::ActorID id = pPhysics->VGetClosestActorHit(
				Vec3(0, 50, 0), actor2StartPosition, pickPosition);
			Assert::AreEqual(pActor2->GetID(), id);
			float distance = (actor2StartPosition - pickPosition).norm();
			Assert::IsTrue(AreEqual(sphereRadius, distance, 0.04f));
		}

		TEST_METHOD(RayTestCanPickADynamicObjectInsideATrigger)
		{
			Vec3 actorStartPosition(100, 50, 0);
			pActor->GetWorldTransform().SetPosition(actorStartPosition);
			pPhysics->VAddBox(pActor, Vec3(40.f, 40.f, 40.f),
				IPhysicsEngine::PhysicsObjectType::TRIGGER);

			auto pGame = GameData::GetInstance();
			auto pActor2 = std::make_shared<GameActor>();
			pGame->AddActor(pActor2);
			pActor2->GetWorldTransform().SetPosition(actorStartPosition);

			float sphereRadius = 10.f;
			pPhysics->VAddSphere(pActor2, sphereRadius,
				IPhysicsEngine::PhysicsObjectType::DYNAMIC, "balsa", "Normal");

			Vec3 pickPosition;
			GameEngine::ActorID id = pPhysics->VGetClosestActorHit(
				Vec3(0, 50, 0), actorStartPosition, pickPosition);
			Assert::AreEqual(pActor2->GetID(), id);
			float distance = (actorStartPosition - pickPosition).norm();
			Assert::IsTrue(AreEqual(sphereRadius, distance, 0.04f));
		}

		TEST_METHOD(RayTestReturnsTheFirstDynamicObjectHit)
		{
			Vec3 actorStartPosition(100, 50, 0);
			pActor->GetWorldTransform().SetPosition(actorStartPosition);
			float sphereRadius = 10.f;
			pPhysics->VAddSphere(pActor, sphereRadius,
				IPhysicsEngine::PhysicsObjectType::DYNAMIC, "balsa", "Normal");

			auto pGame = GameData::GetInstance();
			auto pActor2 = std::make_shared<GameActor>();
			pGame->AddActor(pActor2);
			Vec3 actor2StartPosition(actorStartPosition + Vec3(50, 0, 0));
			pActor2->GetWorldTransform().SetPosition(actor2StartPosition);

			pPhysics->VAddSphere(pActor2, sphereRadius,
				IPhysicsEngine::PhysicsObjectType::DYNAMIC, "balsa", "Normal");

			Vec3 pickPosition;
			GameEngine::ActorID id = pPhysics->VGetClosestActorHit(
				Vec3(0, 50, 0), actor2StartPosition, pickPosition);
			Assert::AreEqual(pActor->GetID(), id);
			float distance = (actorStartPosition - pickPosition).norm();
			Assert::IsTrue(AreEqual(sphereRadius, distance, 0.04f));
		}

		// This object type is declared in the IPhysicsEngine header but is not
		// currently implemented in the BulletPhysics class.
		TEST_METHOD(KinematicObjectsAreUnsupported)
		{
			Assert::ExpectException<std::domain_error>([this] () {
				pPhysics->VAddSphere(pActor, 10.f, IPhysicsEngine::PhysicsObjectType::KINEMATIC);
			});
		}

		TEST_METHOD(UpdatingAConstraintPivotCausesObjectToMoveTowardNewPivot)
		{
			Vec3 actorStartPosition(100, 50, 0);
			pActor->GetWorldTransform().SetPosition(actorStartPosition);
			float sphereRadius = 10.f;
			pPhysics->VAddSphere(pActor, sphereRadius,
				IPhysicsEngine::PhysicsObjectType::DYNAMIC, "balsa", "Normal");

			Vec3 pivotPos(90.f, 50.f, 0.f);
			GameEngine::Physics::ConstraintID constraintId =
				pPhysics->VAddDOF6Constraint(pActor->GetID(), pivotPos);
			SimulateSteps(5);

			Vec3 newPivot(100, 50, 100);
			pPhysics->VUpdateDOF6PivotPoint(constraintId, newPivot);

			SimulateSteps(60); // it takes a while for the DOF6 constrained object to move close enough to the new pivot point

			Vec3 newActorPosition = pActor->GetWorldTransform().GetPosition();
			float distance = (newPivot - newActorPosition).norm();

			Assert::AreNotEqual(newActorPosition, actorStartPosition);
			Assert::IsTrue(AreEqual(10.f, distance, 0.1f));
		}

		TEST_METHOD(IfAngularFactorSetToZeroObjectIsNotAffectedByTorque)
		{
			// Note: an angular velocity can still be set. The effect in that
			// case is that the rotation continues endlessly.
			Vec3 actorStartPosition(100, 50, 0);
			pActor->GetWorldTransform().SetPosition(actorStartPosition);
			float sphereRadius = 10.f;
			pPhysics->VAddSphere(pActor, sphereRadius,
				IPhysicsEngine::PhysicsObjectType::DYNAMIC, "balsa", "Normal");

			pPhysics->VSetAngularFactor(pActor->GetID(), Vec3(0, 0, 0));

			pPhysics->VApplyTorque(pActor->GetID(), Vec3(1.f, 0.f, 0.f), PI);
			Quaternion oldRotation = pActor->GetWorldTransform().GetRotation();
			SimulateSteps(20);

			Assert::AreEqual(oldRotation, pActor->GetWorldTransform().GetRotation());
		}

		TEST_METHOD(TorqueWorksNormallyAfterAngularFactorRestored)
		{
			Vec3 actorStartPosition(100, 50, 0);
			pActor->GetWorldTransform().SetPosition(actorStartPosition);
			float sphereRadius = 10.f;
			pPhysics->VAddSphere(pActor, sphereRadius,
				IPhysicsEngine::PhysicsObjectType::DYNAMIC, "balsa", "Normal");

			Vec3 oldAngularFactor = pPhysics->VGetAngularFactor(pActor->GetID());
			pPhysics->VSetAngularFactor(pActor->GetID(), Vec3(0, 0, 0));
			SimulateSteps(1);

			pPhysics->VSetAngularFactor(pActor->GetID(), oldAngularFactor);
			pPhysics->VApplyTorque(pActor->GetID(), Vec3(-1.f, 0.f, 0.f), PI);
			Vec3 oldRotation = GameEngine::ConvertVector(
				GameEngine::QuaternionToEuler(pActor->GetWorldTransform().GetRotation()));
			SimulateSteps(1);

			Vec3 actorRotation = GameEngine::ConvertVector(
				GameEngine::QuaternionToEuler(pActor->GetWorldTransform().GetRotation()));
			Vec3 difference = Vec3(actorRotation - oldRotation);
			Assert::AreNotEqual(oldRotation, actorRotation);
			Assert::AreEqual(0.f, difference.y());
			Assert::AreEqual(0.f, difference.z());
			Assert::IsTrue(difference.x() > 0.f);
		}

		TEST_METHOD(CannotUpdateConstraintAfterRemoval)
		{
			Vec3 actorStartPosition(100, 50, 0);
			pActor->GetWorldTransform().SetPosition(actorStartPosition);
			float sphereRadius = 10.f;
			pPhysics->VAddSphere(pActor, sphereRadius,
				IPhysicsEngine::PhysicsObjectType::DYNAMIC, "balsa", "Normal");

			pPhysics->VSetGlobalGravity(Vec3(0, 0, 0));
			Vec3 pivotPos(90.f, 50.f, 0.f);
			unsigned int constraintId = pPhysics->VAddDOF6Constraint(pActor->GetID(), pivotPos);

			SimulateSteps(1);
			pPhysics->VRemoveConstraint(constraintId);

			Vec3 newPivotPos(100, 50, 100);
			Vec3 actorPositionAfterConstraintRemoval = pActor->GetWorldTransform().GetPosition();
			pPhysics->VUpdateDOF6PivotPoint(constraintId, newPivotPos);
			SimulateSteps(30);
			Vec3 actorPositionAfterSimulation = pActor->GetWorldTransform().GetPosition();

			Assert::AreEqual(actorPositionAfterSimulation, actorPositionAfterConstraintRemoval);
		}

		TEST_METHOD(CanRemovePhysicsWorldObjectWhileItIsConstrained)
		{
			Vec3 actorStartPosition(100, 50, 0);
			pActor->GetWorldTransform().SetPosition(actorStartPosition);
			float sphereRadius = 10.f;
			pPhysics->VAddSphere(pActor, sphereRadius,
				IPhysicsEngine::PhysicsObjectType::DYNAMIC, "balsa", "Normal");

			pPhysics->VSetGlobalGravity(Vec3(0, 0, 0));
			Vec3 pivotPos(90.f, 50.f, 0.f);
			unsigned int constraintId = pPhysics->VAddDOF6Constraint(pActor->GetID(), pivotPos);

			pPhysics->VRemoveActor(pActor->GetID());
			SimulateSteps(30);
			Assert::AreEqual(actorStartPosition, pActor->GetWorldTransform().GetPosition());
		}

		TEST_METHOD(CannotConstrainAStaticObject)
		{
			Vec3 actorStartPosition(100, 50, 0);
			pActor->GetWorldTransform().SetPosition(actorStartPosition);
			float sphereRadius = 10.f;
			pPhysics->VAddSphere(pActor, sphereRadius,
				IPhysicsEngine::PhysicsObjectType::STATIC);

			pPhysics->VSetGlobalGravity(Vec3(0, 0, 0));
			Vec3 pivotPos(90.f, 50.f, 0.f);
			unsigned int constraintId = pPhysics->VAddDOF6Constraint(pActor->GetID(), pivotPos);
			SimulateSteps(1);

			Vec3 newPivotPos(100, 50, 100);
			pPhysics->VUpdateDOF6PivotPoint(constraintId, newPivotPos);
			SimulateSteps(30);
			Vec3 actorPositionAfterSimulation = pActor->GetWorldTransform().GetPosition();

			Assert::AreEqual(actorPositionAfterSimulation, actorStartPosition);
		}

		TEST_METHOD(BouncyObjectsBounceOffOtherObjects)
		{
			Vec3 actorStartPosition(0, 100, 0);
			pActor->GetWorldTransform().SetPosition(actorStartPosition);
			pPhysics->VAddSphere(pActor, 10.f,
				IPhysicsEngine::PhysicsObjectType::DYNAMIC, "balsa", "Bouncy");

			Vec3 floorStartPosition(0, 0, 0);
			auto pFloor = std::make_shared<GameActor>();
			GameData::GetInstance()->AddActor(pFloor);
			pPhysics->VAddBox(pFloor, Vec3(1000.f, 0.02f, 1000.f),
				IPhysicsEngine::PhysicsObjectType::STATIC, "", "Normal");

			MockEventReceiver collisionListener;
			collisionListener.RegisterTo(GameEngine::Events::EventType::SEPARATION_EVENT, pEvents);

			Vec3 position;
			for (int i = 0; i < 90 && collisionListener.NumValidCallsReceived() < 1; ++i)
			{
				SimulateSteps(1);
				position = pActor->GetWorldTransform().GetPosition();
			}
			Assert::AreEqual(1, collisionListener.NumValidCallsReceived());
		}

		TEST_METHOD(NonBouncyObjectsDoNotBounce)
		{
			Vec3 actorStartPosition(0, 100, 0);
			pActor->GetWorldTransform().SetPosition(actorStartPosition);
			pPhysics->VAddSphere(pActor, 10.f,
				IPhysicsEngine::PhysicsObjectType::DYNAMIC, "balsa", "PlayDough");

			Vec3 floorStartPosition(0, 0, 0);
			auto pFloor = std::make_shared<GameActor>();
			GameData::GetInstance()->AddActor(pFloor);
			pPhysics->VAddBox(pFloor, Vec3(1000.f, 0.02f, 1000.f),
				IPhysicsEngine::PhysicsObjectType::STATIC, "", "Normal");

			MockEventReceiver collisionListener;
			collisionListener.RegisterTo(GameEngine::Events::EventType::SEPARATION_EVENT, pEvents);

			Vec3 position;
			for (int i = 0; i < 90 && collisionListener.NumValidCallsReceived() < 1; ++i)
			{
				SimulateSteps(1);
				position = pActor->GetWorldTransform().GetPosition();
			}
			Assert::AreEqual(0, collisionListener.NumCallsReceived());
		}

		TEST_METHOD(SphereEventuallyStopsRollingWhenBothSphereAndSurfaceHaveFriction)
		{
			Vec3 actorStartPosition(0, 10, 0);
			pActor->GetWorldTransform().SetPosition(actorStartPosition);
			pPhysics->VAddSphere(pActor, 12.f,
				IPhysicsEngine::PhysicsObjectType::DYNAMIC, "Titanium", "Normal");

			Vec3 floorStartPosition(0, 0, 0);
			auto pFloor = std::make_shared<GameActor>();
			GameData::GetInstance()->AddActor(pFloor);
			pPhysics->VAddBox(pFloor, Vec3(2000.f, 0.02f, 2000.f),
				IPhysicsEngine::PhysicsObjectType::STATIC, "", "Normal");

			pPhysics->VApplyForce(pActor->GetID(), Vec3(1.f, 0.f, 0.f), 1.0f); // apply a slight force to get the ball rolling

			SimulateSteps(150);
			Vec3 oldActorPosition = pActor->GetWorldTransform().GetPosition();
			Quaternion oldActorRotation = pActor->GetWorldTransform().GetRotation();
			SimulateSteps(10);

			Assert::AreEqual(oldActorPosition, pActor->GetWorldTransform().GetPosition());
			Assert::AreEqual(oldActorRotation, pActor->GetWorldTransform().GetRotation());
		}

		TEST_METHOD(IfObjectHasNoFrictionItDoesNotStopSlidingOrRolling)
		{
			Vec3 actorStartPosition(0, 10, 0);
			pActor->GetWorldTransform().SetPosition(actorStartPosition);
			pPhysics->VAddSphere(pActor, 12.f,
				IPhysicsEngine::PhysicsObjectType::DYNAMIC, "balsa");

			Vec3 floorStartPosition(0, 0, 0);
			auto pFloor = std::make_shared<GameActor>();
			GameData::GetInstance()->AddActor(pFloor);
			pPhysics->VAddBox(pFloor, Vec3(2000.f, 0.02f, 2000.f),
				IPhysicsEngine::PhysicsObjectType::STATIC, "", "");

			pPhysics->VApplyForce(pActor->GetID(), Vec3(1.f, 0.f, 0.f), 1.f); // apply a slight force to get the ball rolling

			SimulateSteps(500);
			Vec3 oldActorPosition = pActor->GetWorldTransform().GetPosition();
			Quaternion oldActorRotation = pActor->GetWorldTransform().GetRotation();
			SimulateSteps(10);

			Assert::AreNotEqual(oldActorPosition, pActor->GetWorldTransform().GetPosition());
			Assert::AreNotEqual(oldActorRotation, pActor->GetWorldTransform().GetRotation());
		}
	private:
		std::shared_ptr<IPhysicsEngine> pPhysics;
		std::shared_ptr<IEventManager> pEvents;
		std::shared_ptr<GameActor> pActor;
		void SimulateSteps(int steps)
		{
			for (int i = 0; i < steps; ++i)
			{
				pPhysics->VUpdateSimulation(DELTA_TIME_STEP);
				pPhysics->VSyncScene();
				pEvents->VDispatchEvents();
			}
		}
	};
}