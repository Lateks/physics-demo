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
			auto pGame = GameData::GetInstance();
			pGame->SetPhysicsEngine(nullptr);
			pPhysics.reset();
			pGame->RemoveActor(pActor->GetID());
			pActor.reset();
			pEvents.reset();
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
			 * should collide within a second. In practice, Bullet detects
			 * this collision slightly earlier (probably an issue with the
			 * contact processing threshold?).
			 */
			pPhysics->VSetLinearVelocity(pActor->GetID(), distance, 20.f);

			MockEventReceiver receiver;
			receiver.RegisterTo(EventType::COLLISION_EVENT, pGame->GetEventManager());

			SimulateSteps(30);
			Assert::AreEqual(0, receiver.NumCallsReceived());
			SimulateSteps(30);
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

		TEST_METHOD(PickConstraintKeepsObjectAtTheSameDistanceItWasPickedAt)
		{
			Vec3 actorStartPosition(100, 50, 0);
			pActor->GetWorldTransform().SetPosition(actorStartPosition);
			float sphereRadius = 10.f;
			pPhysics->VAddSphere(pActor, sphereRadius,
				IPhysicsEngine::PhysicsObjectType::DYNAMIC, "balsa", "Normal");

			Vec3 oldCameraPos(0, 50, 0);
			Vec3 oldObjectPos(90.f, 50.f, 0.f);
			pPhysics->VAddPickConstraint(pActor->GetID(), oldObjectPos, oldCameraPos);

			Vec3 newCameraPos(10, 50, 50);
			Vec3 newCameraTarget(100, 50, 100);
			auto cameraMoveEvent = std::make_shared<GameEngine::Events::CameraMoveEvent>(
				0u, newCameraPos, newCameraTarget);
			pEvents->VQueueEvent(cameraMoveEvent);

			SimulateSteps(30); // it takes a while for the DOF6 constrained object to move to the new pivot point

			Vec3 newActorPosition = pActor->GetWorldTransform().GetPosition();
			Vec3 oldCameraToActorVector = oldObjectPos - oldCameraPos;
			Vec3 newCameraToTargetVector = newCameraTarget - newCameraPos;
			Vec3 newCameraToActorVector = newActorPosition - newCameraPos;
			Vec3 pickPosition;
			GameEngine::ActorID id = pPhysics->VGetClosestActorHit(
				newCameraPos, newCameraTarget, pickPosition);

			Assert::AreNotEqual(newActorPosition, actorStartPosition);
			Assert::IsTrue(AreEqual(oldCameraToActorVector.norm(),
				newCameraToActorVector.norm() - sphereRadius, 0.5f/WORLD_SCALE));
			Assert::AreEqual(pActor->GetID(), id); // object is now in the direction the camera points to
		}

		// TODO: Test removing a pick constraint and sending camera move events.
		// TODO: Test removal of a physics world object when it is affected by a constraint
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