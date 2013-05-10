#include "Vec4.h"
#include <iostream>

namespace GameEngine
{
	const Vec4 Vec4::White(1.f, 1.f, 1.f, 1.f);
	const Vec4 Vec4::Black(0.f, 0.f, 0.f, 1.f);
	const Vec4 Vec4::Blue(0.f, 0.f, 1.f, 1.f);
	const Vec4 Vec4::Red(1.f, 0.f, 0.f, 1.f);
	const Vec4 Vec4::Green(0.f, 1.f, 0.f, 1.f);

	struct Vec4Data
	{
		Vec4Data() : m_x(0.f), m_y(0.f), m_z(0.f), m_w(0.f) { }
		Vec4Data(float x, float y, float z, float w) : m_x(x), m_y(y), m_z(z), m_w(w) { }
		bool operator==(Vec4Data& other)
		{
			return m_x == other.m_x &&
				m_y == other.m_y &&
				m_z == other.m_z &&
				m_w == other.m_w;
		}
		float m_x;
		float m_y;
		float m_z;
		float m_w;
	};

	Vec4::Vec4() : m_pData(new Vec4Data()) { }

	Vec4::~Vec4() { }

	Vec4::Vec4(float x, float y, float z, float w)
		: m_pData(new Vec4Data(x, y, z, w)) { }

	Vec4::Vec4(const Vec4& other)
		: m_pData(new Vec4Data())
	{
		m_pData->m_x = other.m_pData->m_x;
		m_pData->m_y = other.m_pData->m_y;
		m_pData->m_z = other.m_pData->m_z;
		m_pData->m_w = other.m_pData->m_w;
	}

	Vec4::Vec4(Vec4&& other) : m_pData(std::move(other.m_pData)) { }

	bool Vec4::operator==(const Vec4& other) const
	{
		return *m_pData == *other.m_pData;
	}

	bool Vec4::operator!=(const Vec4& other) const
	{
		return !(*m_pData == *other.m_pData);
	}

	Vec4 Vec4::operator=(const Vec4& other)
	{
		if (this != &other)
		{
			m_pData->m_x = other.m_pData->m_x;
			m_pData->m_y = other.m_pData->m_y;
			m_pData->m_z = other.m_pData->m_z;
			m_pData->m_w = other.m_pData->m_w;
		}
		return *this;
	}

	Vec4 Vec4::operator=(Vec4&& other)
	{
		if (this != &other)
		{
			m_pData = std::move(other.m_pData);
		}
		return *this;
	}

	float Vec4::x() const
	{
		return m_pData->m_x;
	}

	float Vec4::y() const
	{
		return m_pData->m_y;
	}

	float Vec4::z() const
	{
		return m_pData->m_z;
	}

	float Vec4::w() const
	{
		return m_pData->m_w;
	}

	float Vec4::r() const
	{
		return m_pData->m_x;
	}

	float Vec4::g() const
	{
		return m_pData->m_y;
	}

	float Vec4::b() const
	{
		return m_pData->m_z;
	}

	float Vec4::a() const
	{
		return m_pData->m_w;
	}

	std::wostream& operator<<(std::wostream& stream, const Vec4& vec)
	{
		stream << L"("
				<< vec.x() << L" "
				<< vec.y() << L" "
				<< vec.z() << L" "
				<< vec.w() << L")";
		return stream;
	}
}