#ifndef VECTOR_3D_IMPL_H
#define VECTOR_3D_IMPL_H
#include <Eigen/Dense>

struct Vector3DImpl
{
	Vector3DImpl(double x, double y, double z)
		: value(x, y, z) { }
	Vector3DImpl(Eigen::Vector3d val)
		: value(val) { }
	Eigen::Vector3d value;
};

#endif