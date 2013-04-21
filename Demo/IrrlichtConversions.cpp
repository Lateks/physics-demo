#include "IrrlichtConversions.h"
#include "Vec3.h"
#include "Vec4.h"
#include "Mat4.h"

using irr::core::vector3df;
using irr::core::quaternion;
using irr::core::matrix4;

namespace GameEngine
{
	namespace Display
	{
		vector3df ConvertVector(Vec3& vector)
		{
			vector3df converted(vector.x(), vector.y(), vector.z());
			if (vector.GetHandedness() == CSHandedness::RIGHT)
			{
				converted.Z = -converted.Z;
			}
			return converted;
		}

		// Assumes left-handed coordinate system for the input vector.
		Vec3 ConvertVector(vector3df& vector)
		{
			return Vec3(vector.X, vector.Y, -vector.Z);
		}

		quaternion ConvertQuaternion(Quaternion& quat)
		{
			return quaternion(quat.x(), quat.y(), quat.z(), quat.w());
		}

		// TODO: handedness
		matrix4 ConvertProjectionMatrix(Mat4& matrix)
		{
			matrix4 newMatrix;
			for (int i = 0; i < 4; i++)
				for (int j = 0; j < 4; j++)
				{
					float value = matrix.index(i, j);
					newMatrix[i*4 + j] = (float) matrix.index(i, j);
				}
			return newMatrix;
		}
	}
}