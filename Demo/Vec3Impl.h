#ifndef VEC_3_IMPL_H
#define VEC_3_IMPL_H
#include <Eigen/Dense>

namespace GameEngine
{
	namespace LinearAlgebra
	{
		struct Vec3Impl
		{
			Vec3Impl(double x, double y, double z)
				: value(x, y, z) { }
			Vec3Impl(Eigen::Vector3d val)
				: value(val) { }
			Eigen::Vector3d value;
		};
	}
}

#endif