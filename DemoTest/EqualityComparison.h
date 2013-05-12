#pragma once
#include <enginefwd.h>

namespace DemoTest
{
	bool AreEqual(GameEngine::Quaternion& expectedValue, GameEngine::Quaternion& actualValue, float errorTerm);
	bool AreEqual(GameEngine::Vec3& expectedValue, GameEngine::Vec3& actualValue, float errorTerm);

	inline bool AreEqual(float expectedValue, float actualValue, float errorTerm)
	{
		return abs(actualValue - expectedValue) < errorTerm;
	}
}