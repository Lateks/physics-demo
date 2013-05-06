#include "Vec4.h"
#include <iostream>

namespace GameEngine
{
	Vec4::Vec4(const Vec4& other)
	{
		if (this != &other)
		{
			m_x = other.m_x;
			m_y = other.m_y;
			m_z = other.m_z;
			m_w = other.m_w;
		}
	}

	bool Vec4::operator==(const Vec4& other) const
	{
		return m_x == other.m_x &&
				m_y == other.m_y &&
				m_z == other.m_z &&
				m_w == other.m_w;
	}

	bool Vec4::operator!=(const Vec4& other) const
	{
		return m_x != other.m_x ||
				m_y != other.m_y ||
				m_z != other.m_z ||
				m_w != other.m_w;
	}

	Vec4 Vec4::operator=(const Vec4& other)
	{
		if (this != &other)
		{
			m_x = other.m_x;
			m_y = other.m_y;
			m_z = other.m_z;
			m_w = other.m_w;
		}
		return *this;
	}

	std::ostream& operator<<(std::ostream& stream, const Vec4& vec)
	{
		stream << "("
				<< vec.x() << " "
				<< vec.y() << " "
				<< vec.z() << " "
				<< vec.w() << ")";
		return stream;
	}
}