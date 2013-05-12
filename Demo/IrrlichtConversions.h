#pragma once

#include "enginefwd.h"
#include <irrlicht.h>

namespace GameEngine
{
	irr::core::vector3df ConvertVector(const Vec3& vector);

	// This assumes a left-handed coordinate system for the input vector.
	Vec3 ConvertVector(const irr::core::vector3df& vector);

	irr::core::quaternion ConvertQuaternion(const Quaternion& quat);

	Quaternion ConvertQuaternion(const irr::core::quaternion& quat);

	// return value is in radians
	irr::core::vector3df QuaternionToEuler(const irr::core::quaternion& quat);

	// return value is in radians
	irr::core::vector3df QuaternionToEuler(const Quaternion& quat);

	Quaternion EulerToQuaternion(const irr::core::vector3df& euler);

	irr::video::SColorf ConvertRGBAColorToSColorf(const RGBAColor& color);

	irr::video::SColor ConvertRGBAColorToSColor(const RGBAColor& color);
}