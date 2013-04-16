#include "BulletConversions.h"

namespace GameEngine
{
	namespace Physics
	{
		Vec3 btVector3_to_Vec3(const btVector3& vec)
		{
			return Vec3(vec.x(), vec.y(), vec.z());
		}

		btVector3 Vec3_to_btVector3(const Vec3& vec)
		{
			return btVector3(vec.x(), vec.y(), vec.z());
		}

		btVector3 Vec4_to_btVector3(const Vec4& vec)
		{
			btVector3 converted(vec.x(), vec.y(), vec.z());
			converted[3] = vec.w();
			return converted;
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