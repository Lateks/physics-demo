#include "BulletConversions.h"

namespace GameEngine
{
	namespace PhysicsEngine
	{
		LinearAlgebra::Vec3 btVector3_to_Vec3(const btVector3& vec)
		{
			return LinearAlgebra::Vec3(vec.x(), vec.y(), vec.z());
		}

		btVector3 Vec3_to_btVector3(const LinearAlgebra::Vec3& vec)
		{
			return btVector3(vec.x(), vec.y(), vec.z());
		}

		LinearAlgebra::Quaternion btQuaternion_to_Quaternion(const btQuaternion& quat)
		{
			return LinearAlgebra::Quaternion(quat.x(), quat.y(), quat.z(), quat.w());
		}

		btQuaternion Quaternion_to_btQuaternion(const LinearAlgebra::Quaternion& quat)
		{
			return btQuaternion(quat.x(), quat.y(), quat.z(), quat.w());
		}
	}
}