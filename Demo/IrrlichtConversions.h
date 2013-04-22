#ifndef IRRLICHT_CONVERSIONS_H
#define IRRLICHT_CONVERSIONS_H

#include "enginefwd.h"
#include <irrlicht.h>

namespace GameEngine
{
	namespace Display
	{
		irr::core::vector3df ConvertVector(Vec3& vector);
		Vec3 ConvertVector(irr::core::vector3df& vector);
		irr::core::quaternion ConvertQuaternion(Quaternion& quat);
		Quaternion ConvertQuaternion(irr::core::quaternion& quat);
		irr::core::matrix4 ConvertProjectionMatrix(Mat4& matrix);
	}
}

#endif