#include "CppUnitTest.h"
#include "ToString.h"
#include <Vec3.h>
#include <Vec4.h>

using namespace Microsoft::VisualStudio::CppUnitTestFramework;

namespace DemoTest
{		
	TEST_CLASS(VectorTest)
	{
	public:

		TEST_METHOD(Vec3Creation)
		{
			GameEngine::Vec3 vector(15, 6, 1);
			Assert::AreEqual(15.f, vector.x());
			Assert::AreEqual(6.f, vector.y());
			Assert::AreEqual(1.f, vector.z());
		}

		TEST_METHOD(Vec3DefaultHandedness)
		{
			GameEngine::Vec3 vector;
			Assert::AreEqual(vector.GetHandedness(), GameEngine::CSHandedness::RIGHT);
		}

		TEST_METHOD(Vec3EqualityComparisonTakesHandednessIntoAccount)
		{
			GameEngine::Vec3 rhVector(1, 1, 1, GameEngine::CSHandedness::RIGHT);
			GameEngine::Vec3 rhVector2(1, 1, 1, GameEngine::CSHandedness::RIGHT);
			GameEngine::Vec3 lhVector(1, 1, 1, GameEngine::CSHandedness::LEFT);
			Assert::AreEqual(rhVector, rhVector2);
			Assert::AreNotEqual(rhVector, lhVector);
		}

		TEST_METHOD(Vec3NonEqualityComparisonTakesHandednessIntoAccount)
		{
			GameEngine::Vec3 rhVector(1, 1, 1, GameEngine::CSHandedness::RIGHT);
			GameEngine::Vec3 rhVector2(1, 1, 1, GameEngine::CSHandedness::RIGHT);
			GameEngine::Vec3 lhVector(1, 1, 1, GameEngine::CSHandedness::LEFT);
			Assert::IsFalse(rhVector != rhVector2);
			Assert::IsTrue(rhVector != lhVector);
		}

		TEST_METHOD(Vec3FlipHandednessFromRightToLeft)
		{
			GameEngine::Vec3 rhVector(1, 1, 1);
			GameEngine::Vec3 lhVector = rhVector.FlipHandedness();
			GameEngine::Vec3 expected(1, 1, -1, GameEngine::CSHandedness::LEFT);
			Assert::AreEqual(expected, lhVector);
		}

		TEST_METHOD(Vec3FlipHandednessFromLeftToRight)
		{
			GameEngine::Vec3 lhVector(1, 1, 1, GameEngine::CSHandedness::LEFT);
			GameEngine::Vec3 rhVector = lhVector.FlipHandedness();
			GameEngine::Vec3 expected(1, 1, -1, GameEngine::CSHandedness::RIGHT);
			Assert::AreEqual(expected, rhVector);
		}

		TEST_METHOD(Vec3HandednessFlipHasNoEffectWhenVectorHasNoHandedness)
		{
			GameEngine::Vec3 vector(1, 1, 1, GameEngine::CSHandedness::NONE); // this might be a vector used e.g. for scaling
			GameEngine::Vec3 flipped = vector.FlipHandedness();
			Assert::AreEqual(flipped.GetHandedness(), GameEngine::CSHandedness::NONE);
			Assert::AreEqual(vector, flipped);
		}

		TEST_METHOD(Vec3CanDoBasicVectorAddition)
		{
			GameEngine::Vec3 result = GameEngine::Vec3(1, 2, 3) + GameEngine::Vec3(2, 4, 5);
			GameEngine::Vec3 expected(3, 6, 8);
			Assert::AreEqual(expected, result);
		}

		TEST_METHOD(Vec3CanDoBasicScalarMultiplication)
		{
			GameEngine::Vec3 result = GameEngine::Vec3(1, 2, 3) * 2;
			GameEngine::Vec3 expected(2, 4, 6);
			Assert::AreEqual(expected, result);
		}

		TEST_METHOD(Vec4Creation)
		{
			GameEngine::RGBAColor rgbaColor(0.2f, 0.5f, 0.7f, 1.f);
			Assert::AreEqual(0.2f, rgbaColor.r());
			Assert::AreEqual(0.5f, rgbaColor.g());
			Assert::AreEqual(0.7f, rgbaColor.b());
			Assert::AreEqual(1.f, rgbaColor.a());
		}

		TEST_METHOD(Vec4MemberAliases)
		{
			GameEngine::RGBAColor rgbaColor(0.2f, 0.5f, 0.7f, 1.f);
			Assert::AreEqual(rgbaColor.x(), rgbaColor.r());
			Assert::AreEqual(rgbaColor.y(), rgbaColor.g());
			Assert::AreEqual(rgbaColor.z(), rgbaColor.b());
			Assert::AreEqual(rgbaColor.w(), rgbaColor.a());
		}
	};
}