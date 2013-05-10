#pragma once

#include "enginefwd.h"
#include <Eigen\Dense>
#include <iosfwd>

namespace GameEngine
{
	enum class CSHandedness
	{
		NONE,
		LEFT,
		RIGHT
	};

	class Vec3 : public Eigen::Vector3f
	{
	public:
		Vec3()
			: Eigen::Vector3f(0.f, 0.f, 0.f), m_handedness(CSHandedness::RIGHT) { };
		Vec3(float x, float y, float z, CSHandedness handedness = CSHandedness::RIGHT)
			: Eigen::Vector3f(x, y, z), m_handedness(handedness) { };
		Vec3(Eigen::Vector3f vector, CSHandedness handedness = CSHandedness::RIGHT)
			: Eigen::Vector3f(vector), m_handedness(handedness) { };
		virtual ~Vec3() { };
		bool operator==(const Vec3& other) const;
		CSHandedness GetHandedness() const
		{
			return m_handedness;
		}
		Vec3 FlipHandedness()
		{
			if (m_handedness == CSHandedness::NONE)
				return *this;
			CSHandedness newHandedness = m_handedness == CSHandedness::RIGHT ?
				CSHandedness::LEFT : CSHandedness::RIGHT;
			return Vec3(x(), y(), -z(), newHandedness);
		}
	private:
		CSHandedness m_handedness;
	};

	std::wostream& operator<<(std::wostream& stream, const CSHandedness& handedness);

	std::wostream& operator<<(std::wostream& stream, const Vec3& vec);
}