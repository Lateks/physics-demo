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
#include <irrlicht.h> // Use irrlicht types to get conversions between quaternions and euler angles.
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
	const float DELTA_TIME_STEP = 1/60.f;
	const float PI = 3.14159f;
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

			pPhysics->VUpdateSimulation(DELTA_TIME_STEP);
			pPhysics->VSyncScene();

			Vec3 actorPosition = pActor->GetWorldTransform().GetPosition();
			Vec3 gravityVector = (actorPosition - actorStartPosition).normalized();
			Assert::AreEqual(Vec3(0, -1, 0), gravityVector);
		}

		TEST_METHOD(ApplyingAForce)
		{
			Vec3 actorStartPosition(0, 100, 0);
			pActor->GetWorldTransform().SetPosition(actorStartPosition);
			pPhysics->VAddSphere(pActor, 10.f,
				GameEngine::Physics::IPhysicsEngine::PhysicsObjectType::DYNAMIC,
				"balsa", "Normal");

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
				GameEngine::Physics::IPhysicsEngine::PhysicsObjectType::DYNAMIC,
				"balsa", "Normal");

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
			irr::core::vector3df startRotation;
			ConvertQuaternion(pActor->GetWorldTransform().GetRotation()).toEuler(startRotation);
			pPhysics->VAddSphere(pActor, 10.f,
				GameEngine::Physics::IPhysicsEngine::PhysicsObjectType::DYNAMIC,
				"balsa", "Normal");

			// Set an angular velocity about the x axis in radians per second.
			pPhysics->VSetAngularVelocity(pActor->GetID(), Vec3(1, 0, 0), PI);
			pPhysics->VSetGlobalGravity(Vec3(0, 0, 0)); // disregard gravity

			pPhysics->VUpdateSimulation(DELTA_TIME_STEP);
			pPhysics->VSyncScene();

			irr::core::vector3df actorRotation;
			ConvertQuaternion(pActor->GetWorldTransform().GetRotation()).toEuler(actorRotation);
			irr::core::vector3df difference = actorRotation-startRotation;
			Assert::AreEqual(0.f, difference.Y);
			Assert::AreEqual(0.f, difference.Z);
			Assert::IsTrue(AreEqual(PI*DELTA_TIME_STEP, difference.X, 0.00001f));
		}

		TEST_METHOD(ApplyingATorque)
		{
			Vec3 actorStartPosition(0, 100, 0);
			pActor->GetWorldTransform().SetPosition(actorStartPosition);
			irr::core::vector3df startRotation;
			ConvertQuaternion(pActor->GetWorldTransform().GetRotation()).toEuler(startRotation);
			pPhysics->VAddSphere(pActor, 10.f,
				GameEngine::Physics::IPhysicsEngine::PhysicsObjectType::DYNAMIC,
				"balsa", "Normal");

			pPhysics->VApplyTorque(Vec3(1, 0, 0), PI, pActor->GetID());
			pPhysics->VSetGlobalGravity(Vec3(0, 0, 0)); // disregard gravity

			pPhysics->VUpdateSimulation(DELTA_TIME_STEP);
			pPhysics->VSyncScene();

			irr::core::vector3df actorRotation;
			ConvertQuaternion(pActor->GetWorldTransform().GetRotation()).toEuler(actorRotation);
			irr::core::vector3df difference = actorRotation-startRotation;
			Assert::AreEqual(0.f, difference.Y);
			Assert::AreEqual(0.f, difference.Z);
			Assert::IsTrue(difference.X > 0.f);
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

		// TODO: Test effects of different materials?
		// TODO: Test collision events.
		// TODO: Test separation events.
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
	private:
		std::shared_ptr<IPhysicsEngine> pPhysics;
		std::shared_ptr<IEventManager> pEvents;
		std::shared_ptr<GameActor> pActor;
		irr::core::quaternion ConvertQuaternion(Quaternion& quat)
		{
			return irr::core::quaternion(quat.x(), quat.y(), quat.z(), quat.w());
		}
	};
}