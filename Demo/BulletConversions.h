#pragma once

#include "Vec3.h"
#include "Vec4.h"
#include <btBulletCollisionCommon.h>

namespace GameEngine
{
	namespace Physics
	{
		Vec3 btVector3_to_Vec3(const btVector3& vec, float worldScale = 1.f);
		btVector3 Vec3_to_btVector3(const Vec3& vec, float worldScale = 1.f);
		btVector3 Vec4_to_btVector3(const Vec4& vec);

		Quaternion btQuaternion_to_Quaternion(const btQuaternion& quat);
		btQuaternion Quaternion_to_btQuaternion(const Quaternion& quat);
	}
}