#include "IrrlichtConversions.h"
#include "Vec3.h"
#include "Vec4.h"
#include <irrlicht.h>

using irr::u32;
using irr::core::vector3df;
using irr::core::quaternion;
using irr::video::SColor;
using irr::video::SColorf;

namespace
{
	const unsigned int RGB_MAX = 255;
}

namespace GameEngine
{
	vector3df ConvertVector(const Vec3& vector)
	{
		vector3df converted(vector.x(), vector.y(), vector.z());
		if (vector.GetHandedness() == CSHandedness::RIGHT)
		{
			converted.Z = -converted.Z;
		}
		return converted;
	}

	Vec3 ConvertVector(const vector3df& vector)
	{
		return Vec3(vector.X, vector.Y, -vector.Z);
	}

	quaternion ConvertQuaternion(const Quaternion& quat)
	{
		return quaternion(quat.x(), quat.y(), quat.z(), quat.w());
	}

	Quaternion ConvertQuaternion(const quaternion& quat)
	{
		return Quaternion(quat.X, quat.Y, quat.Z, quat.W);
	}

	vector3df QuaternionToEuler(const quaternion& quat)
	{
		vector3df eulerRot;
		quat.toEuler(eulerRot);
		return eulerRot;
	}

	vector3df QuaternionToEuler(const Quaternion& quat)
	{
		return QuaternionToEuler(ConvertQuaternion(quat));
	}

	Quaternion EulerToQuaternion(const vector3df& euler)
	{
		return ConvertQuaternion(quaternion(euler * irr::core::DEGTORAD));
	}

	SColorf ConvertRGBAColorToSColorf(const RGBAColor& color)
	{
		return SColorf(color.r(), color.g(), color.b(), color.a());
	}

	SColor ConvertRGBAColorToSColor(const RGBAColor& color)
	{
		return SColor((u32) color.a() * RGB_MAX,
			(u32) color.r() * RGB_MAX, (u32) color.g() * RGB_MAX, (u32) color.b() * RGB_MAX);
	}
}