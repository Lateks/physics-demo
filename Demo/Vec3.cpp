#include "Vec3.h"
#include <iostream>

namespace GameEngine
{
	Vec3::Vec3(const Vec3& other)
	{
		if (this != &other)
		{
			m_x = other.m_x;
			m_y = other.m_y;
			m_z = other.m_z;
		}
	}

	bool Vec3::operator==(const Vec3& other) const
	{
		return m_x == other.m_x &&
				m_y == other.m_y &&
				m_z == other.m_z;
	}

	bool Vec3::operator!=(const Vec3& other) const
	{
		return m_x != other.m_x ||
				m_y != other.m_y ||
				m_z != other.m_z;
	}

	Vec3 Vec3::operator=(const Vec3& other)
	{
		if (this != &other)
		{
			m_x = other.m_x;
			m_y = other.m_y;
			m_z = other.m_z;
		}
		return *this;
	}

	float Vec3::x() const
	{
		return m_x;
	}

	float Vec3::y() const
	{
		return m_y;
	}

	float Vec3::z() const
	{
		return m_z;
	}

	std::ostream& operator<<(std::ostream& stream, const Vec3& vec)
	{
		stream << "("
				<< vec.x() << " "
				<< vec.y() << " "
				<< vec.z() << ")";
		return stream;
	}
}