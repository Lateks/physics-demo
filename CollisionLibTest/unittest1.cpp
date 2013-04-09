#include "stdafx.h"
#include "CppUnitTest.h"
#include "ToStringSpecializations.h"
#include <IPositionedObject.h>
#include <SphereShape.h>
#include <CollisionMath.h>
#include <cmath>

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

	TEST_CLASS(GJKTest)
	{
	public:

		TEST_METHOD(SphereShapeTest1)
		{
			TestSphereActor sphereobj(3, 3, 0);
			SphereShape sphere(3.0, &sphereobj);

			Vector3D searchDirection(1, 0, 0);
			Point3D expected(6, 3, 0);
			Point3D result = sphere.GetFarthestPointInDirection(searchDirection);
			Assert::AreEqual(expected, result);
		}

		TEST_METHOD(SphereShapeTest2)
		{
			TestSphereActor sphereobj(5, 0, 0);
			SphereShape sphere(3.0, &sphereobj);

			Vector3D searchDirection(1, 1, 1);
			double normVecElem = 1/sqrt(3);
			double offset = 3*normVecElem;
			Point3D expected(5 + offset, offset, offset);
			Point3D result = sphere.GetFarthestPointInDirection(searchDirection);
			Assert::AreEqual(expected, result);
		}

		TEST_METHOD(PenetratingSpheres)
		{
			TestSphereActor sphereobj1(3, 3, 0);
			SphereShape sphere1(3.0, &sphereobj1);

			TestSphereActor sphereobj2(0, 0, 0);
			SphereShape sphere2(3.0, &sphereobj2);

			Assert::IsTrue(GJKCollide(sphere1, sphere2));
		}

	};
}