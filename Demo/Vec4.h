#ifndef VEC_4_H
#define VEC_4_H

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
		float x() const;
		float y() const;
		float z() const;
		float w() const;
	private:
		float m_x;
		float m_y;
		float m_z;
		float m_w;
	};
}

#endif