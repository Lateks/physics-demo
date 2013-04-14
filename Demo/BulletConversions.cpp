#include "BulletConversions.h"

namespace GameEngine
{
	namespace PhysicsEngine
	{
		Vec3 btVector3_to_Vec3(const btVector3& vec)
		{
			return Vec3(vec.x(), vec.y(), vec.z());
		}

		btVector3 Vec3_to_btVector3(const Vec3& vec)
		{
			return btVector3(vec.x(), vec.y(), vec.z());
		}

		Quaternion btQuaternion_to_Quaternion(const btQuaternion& quat)
		{
			return Quaternion(quat.x(), quat.y(), quat.z(), quat.w());
		}

		btQuaternion Quaternion_to_btQuaternion(const Quaternion& quat)
		{
			return btQuaternion(quat.x(), quat.y(), quat.z(), quat.w());
		}
	}
}