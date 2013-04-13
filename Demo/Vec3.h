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
			Vec3()
				: m_x(0), m_y(0), m_z(0) { }
			Vec3(float x, float y, float z)
				: m_x(x), m_y(y), m_z(z) { }
			Vec3(const Vec3& other);
			~Vec3() { };
			Vec3 operator=(const Vec3& other);
			bool operator==(const Vec3& other) const;
			bool operator!=(const Vec3& other) const;
			float x() const;
			float y() const;
			float z() const;
		private:
			float m_x;
			float m_y;
			float m_z;
		};

		std::ostream& operator<<(std::ostream& stream, const Vec3& vec);
	}
}

#endif