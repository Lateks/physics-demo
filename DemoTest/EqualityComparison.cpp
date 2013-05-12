#include "EqualityComparison.h"
#include <Vec3.h>
#include <Vec4.h>

namespace DemoTest
{
	bool AreEqual(GameEngine::Quaternion& expectedValue, GameEngine::Quaternion& actualValue, float errorTerm)
	{
		return AreEqual(expectedValue.x(), actualValue.x(), errorTerm)
			&& AreEqual(expectedValue.y(), actualValue.y(), errorTerm)
			&& AreEqual(expectedValue.z(), actualValue.z(), errorTerm)
			&& AreEqual(expectedValue.w(), actualValue.w(), errorTerm);
	}

	bool AreEqual(GameEngine::Vec3& expectedValue, GameEngine::Vec3& actualValue, float errorTerm)
	{
		return AreEqual(expectedValue.x(), actualValue.x(), errorTerm)
			&& AreEqual(expectedValue.y(), actualValue.y(), errorTerm)
			&& AreEqual(expectedValue.z(), actualValue.z(), errorTerm);
	}
}