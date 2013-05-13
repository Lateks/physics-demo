#pragma once

#include "enginefwd.h"
#include <Eigen\Dense>
#include <iosfwd>

namespace GameEngine
{
	class Vec3 : public Eigen::Vector3f
	{
	public:
		Vec3()
			: Eigen::Vector3f(0.f, 0.f, 0.f) { };
		Vec3(float x, float y, float z)
			: Eigen::Vector3f(x, y, z) { };
		Vec3(Eigen::Vector3f vector)
			: Eigen::Vector3f(vector) { };
		virtual ~Vec3() { };
	};

	std::wostream& operator<<(std::wostream& stream, const Vec3& vec);
}