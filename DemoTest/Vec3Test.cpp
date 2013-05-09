#include "CppUnitTest.h"
#include <Vec3.h>

using namespace Microsoft::VisualStudio::CppUnitTestFramework;

namespace DemoTest
{		
	TEST_CLASS(UnitTest1)
	{
	public:
		
		TEST_METHOD(TestVectorCreation)
		{
			GameEngine::Vec3 vector(15, 6, 1);
			Assert::AreEqual(15.f, vector.x());
			Assert::AreEqual(6.f, vector.y());
			Assert::AreEqual(1.f, vector.z());
		}

	};
}