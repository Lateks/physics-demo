#include "stdafx.h"
#include "CppUnitTest.h"
#include <IPositionedObject.h>
#include <SphereShape.h>
#include <CollisionMath.h>

using namespace Microsoft::VisualStudio::CppUnitTestFramework;

namespace CollisionLibTest
{
	struct TestSphereActor : public IPositionedObject
	{
		TestSphereActor() : Position(0, 0, 0) { }
		TestSphereActor(double x, double y, double z)
			: Position(x, y, z) { }
		Vector3D Position;
		Vector3D GetWorldLocation() { return Position; }
	};

	TEST_CLASS(UnitTest1)
	{
	public:
		
		TEST_METHOD(PenetratingSpheres)
		{
			auto sphereobj1 = new TestSphereActor(2, 2, 2);
			SphereShape sphere1(5.0, sphereobj1);

			auto sphereobj2 = new TestSphereActor(0, 4, 0);
			SphereShape sphere2(4.0, sphereobj2);

			Assert::IsTrue(GJKCollide(sphere1, sphere2));
		}

	};
}