#ifndef VEC_3_H
#define VEC_3_H

#include "enginefwd.h"
#include <iosfwd>

namespace GameEngine
{
	namespace LinearAlgebra
	{
		class Vec3
		{
		public:
			Vec3();
			Vec3(double x, double y, double z);
			Vec3(const Vec3& other);
			Vec3(Vec3&& other);
			~Vec3();
			Vec3 operator=(const Vec3& other);
			Vec3 operator=(Vec3&& other);
			bool operator==(const Vec3& other) const;
			Vec3 operator+(const Vec3& other) const;
			Vec3 operator-(const Vec3& other) const;
			Vec3 operator*(double scalar) const;
			Vec3 operator-() const;
			Vec3 cross(const Vec3& other) const;
			double dot(const Vec3& other) const;
			Vec3 normalize() const;
			double getX() const;
			double getY() const;
			double getZ() const;
		private:
			Vec3Impl *pImpl;
		};

		Vec3 operator*(double scalar, const Vec3& vec);
		std::ostream& operator<<(std::ostream& stream, const Vec3& vec);
	}
}

#endif