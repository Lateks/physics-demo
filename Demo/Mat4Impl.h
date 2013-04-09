#ifndef MAT_4_IMPL_H
#define MAT_4_IMPL_H
#include <Eigen/Dense>

namespace GameEngine
{
	namespace LinearAlgebra
	{
		struct Mat4Impl
		{
			Mat4Impl()
				: value() { }
			Mat4Impl(Eigen::Matrix4d val)
				: value(val) { }
			Eigen::Matrix4d value;
		};
	}
}

#endif