#pragma once
#include <memory>

namespace GameEngine
{
	struct Vec4Data;

	class Vec4
	{
	public:
		Vec4();
		Vec4(float x, float y, float z, float w);
		Vec4(const Vec4& other);
		Vec4(Vec4&& other);
		~Vec4();
		Vec4 operator=(const Vec4& other);
		Vec4 operator=(Vec4&& other);
		bool operator==(const Vec4& other) const;
		bool operator!=(const Vec4& other) const;

		float x() const;
		float y() const;
		float z() const;
		float w() const;

		float r() const;
		float g() const;
		float b() const;
		float a() const;

		static const Vec4 White;
		static const Vec4 Black;
		static const Vec4 Blue;
		static const Vec4 Red;
		static const Vec4 Green;
	private:
		std::unique_ptr<Vec4Data> m_pData;
	};
}