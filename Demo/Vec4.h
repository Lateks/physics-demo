#pragma once

namespace GameEngine
{
	class Vec4
	{
	public:
		Vec4()
			: m_x(0), m_y(0), m_z(0), m_w(0) { }
		Vec4(float x, float y, float z, float w)
			: m_x(x), m_y(y), m_z(z), m_w(w) { }
		Vec4(const Vec4& other);
		~Vec4() { };
		Vec4 operator=(const Vec4& other);
		bool operator==(const Vec4& other) const;
		bool operator!=(const Vec4& other) const;

		float x() const
		{
			return m_x;
		}

		float y() const
		{
			return m_y;
		}

		float z() const
		{
			return m_z;
		}

		float w() const
		{
			return m_w;
		}

		float r() const
		{
			return m_x;
		}

		float g() const
		{
			return m_y;
		}

		float b() const
		{
			return m_z;
		}

		float a() const
		{
			return m_w;
		}

		static const Vec4 White;
		static const Vec4 Black;
		static const Vec4 Blue;
		static const Vec4 Red;
		static const Vec4 Green;
	private:
		float m_x;
		float m_y;
		float m_z;
		float m_w;
	};
}