#include "CppUnitTest.h"
#include "ToString.h"
#include <IrrlichtDisplay.h>
#include <Vec3.h>
#include <Vec4.h>
#include <vector>

using namespace Microsoft::VisualStudio::CppUnitTestFramework;
using namespace GameEngine::Display;
using GameEngine::Vec3;
using GameEngine::Quaternion;

namespace DemoTest
{		
	TEST_CLASS(IrrlichtDisplayTest)
	{
	public:
		TEST_METHOD_INITIALIZE(InitializeIrrlichtDisplay)
		{
			pDisplay = new IrrlichtDisplay();
			pDisplay->VSetupAndOpenWindow(0, 0, DriverType::NO_WINDOW, CameraType::FPS_WASD);
		}

		TEST_METHOD_CLEANUP(CleanUpIrrlichtDisplay)
		{
			delete pDisplay;
		}

		TEST_METHOD(IrrlichtCursorIsVisibleByDefault)
		{
			Assert::IsTrue(pDisplay->VCursorVisible());
		}

		TEST_METHOD(IrrlichtCursorCanBeHidden)
		{
			pDisplay->VHideCursor();
			Assert::IsFalse(pDisplay->VCursorVisible());
		}

		TEST_METHOD(IrrlichtCursorCanBeRestored)
		{
			pDisplay->VHideCursor();
			pDisplay->VShowCursor();
			Assert::IsTrue(pDisplay->VCursorVisible());
		}

		TEST_METHOD(IrrlichtCameraIsAtOriginByDefault)
		{
			Assert::AreEqual(Vec3(0, 0, 0), pDisplay->VGetCameraPosition());
		}

		TEST_METHOD(CanSetIrrlichtCameraPosition)
		{
			Vec3 newPosition(100, 100, -100);
			pDisplay->VSetCameraPosition(newPosition);
			Assert::AreEqual(newPosition, pDisplay->VGetCameraPosition());
		}

		TEST_METHOD(CanSetIrrlichtCameraRotation)
		{
			Quaternion expectedRotation(0, 1, 0, 0);
			pDisplay->VSetCameraRotation(expectedRotation);
			Quaternion actualRotation = pDisplay->VGetCameraRotation();
			Assert::IsTrue(AreEqual(expectedRotation, actualRotation, 0.0001f)); // the result is not exact due to conversions to Euler angles and back
		}

		TEST_METHOD(CanSetIrrlichtCameraUpVector)
		{
			Vec3 newUpVector(0, 50, 50);
			pDisplay->VSetCameraUpVector(newUpVector);
			Assert::AreEqual(newUpVector, pDisplay->VGetCameraUpVector());
		}

		TEST_METHOD(CanSetIrrlichtCameraFOV)
		{
			std::vector<float> testValues;
			testValues.push_back(75.f); // FFUUU, no initializer lists in MSVC.
			testValues.push_back(90.f);
			testValues.push_back(120.f);
			testValues.push_back(50.f);
			float epsilon = 0.00001f;

			std::for_each(testValues.begin(), testValues.end(),
				[this, epsilon] (float newFovDegrees)
			{
				pDisplay->VSetCameraFOV(newFovDegrees);
				Assert::IsTrue(AreEqual(newFovDegrees, pDisplay->VGetCameraFOV(), epsilon));
			});
		}

		TEST_METHOD(CanSetIrrlichtCameraTarget)
		{
			Vec3 newTarget(100, 50, 0);
			pDisplay->VSetCameraTarget(newTarget);
			Assert::AreEqual(newTarget, pDisplay->VGetCameraTarget());
		}

		TEST_METHOD(CanSetCameraNearPlaneDistance)
		{
			float newDistance = 1.f;
			pDisplay->VSetCameraNearPlaneDistance(newDistance);
			Assert::AreEqual(newDistance, pDisplay->VGetCameraNearPlaneDistance());
		}

		TEST_METHOD(CanSetCameraFarPlaneDistance)
		{
			float newDistance = 2000.f;
			pDisplay->VSetCameraFarPlaneDistance(newDistance);
			Assert::AreEqual(newDistance, pDisplay->VGetCameraFarPlaneDistance());
		}

		TEST_METHOD(CanGetCameraRightVectorWhenLookingTowardNegativeX)
		{
			Vec3 cameraPos(100, 0, 50);
			Vec3 cameraTarget(0, 0, 50);
			pDisplay->VSetCameraPosition(cameraPos);
			pDisplay->VSetCameraTarget(cameraTarget);

			Vec3 negativeZAxis(0, 0, -1);
			Assert::AreEqual(negativeZAxis, pDisplay->VGetCameraRightVector());
		}

		TEST_METHOD(CanGetCameraRightVectorWhenLookingTowardPositiveX)
		{
			Vec3 cameraPos(50, 0, 50);
			Vec3 cameraTarget(100, 0, 50);
			pDisplay->VSetCameraPosition(cameraPos);
			pDisplay->VSetCameraTarget(cameraTarget);

			Vec3 positiveZAxis(0, 0, 1);
			Assert::AreEqual(positiveZAxis, pDisplay->VGetCameraRightVector());
		}

		TEST_METHOD(CanGetCameraRightVectorWhenLookingTowardNegativeZ)
		{
			Vec3 cameraPos(50, 0, 50);
			Vec3 cameraTarget(50, 0, 0);
			pDisplay->VSetCameraPosition(cameraPos);
			pDisplay->VSetCameraTarget(cameraTarget);

			Vec3 positiveXAxis(1, 0, 0);
			Assert::AreEqual(positiveXAxis, pDisplay->VGetCameraRightVector());
		}

		TEST_METHOD(CanGetCameraRightVectorWhenLookingTowardPositiveZ)
		{
			Vec3 cameraPos(50, 10, -50);
			Vec3 cameraTarget(50, 10, 50);
			pDisplay->VSetCameraPosition(cameraPos);
			pDisplay->VSetCameraTarget(cameraTarget);

			Vec3 negativeXAxis(-1, 0, 0);
			Assert::AreEqual(negativeXAxis, pDisplay->VGetCameraRightVector());
		}

		TEST_METHOD(LoadingANonExistentTexture)
		{
			Assert::AreEqual(0u, pDisplay->VLoadTexture("foo.png"));
		}

		TEST_METHOD(LoadingAValidTexture)
		{
			Assert::AreEqual(1u, pDisplay->VLoadTexture("..\\assets\\woodbox2.jpg"));
		}
	private:
		IDisplay *pDisplay;
		bool AreEqual(Quaternion expectedValue, Quaternion actualValue, float errorTerm)
		{
			return AreEqual(expectedValue.x(), actualValue.x(), errorTerm)
				&& AreEqual(expectedValue.y(), actualValue.y(), errorTerm)
				&& AreEqual(expectedValue.z(), actualValue.z(), errorTerm)
				&& AreEqual(expectedValue.w(), actualValue.w(), errorTerm);
		}
		bool AreEqual(float expectedValue, float actualValue, float errorTerm)
		{
			return abs(actualValue - expectedValue) < errorTerm;
		}
	};
}