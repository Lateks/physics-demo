#include "Vec4.h"
#include <iostream>

namespace GameEngine
{
	namespace LinearAlgebra
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

		float Vec4::x() const
		{
			return m_x;
		}

		float Vec4::y() const
		{
			return m_y;
		}

		float Vec4::z() const
		{
			return m_z;
		}

		float Vec4::w() const
		{
			return m_w;
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
}