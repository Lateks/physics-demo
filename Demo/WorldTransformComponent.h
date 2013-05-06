#pragma once

#include "enginefwd.h"
#include "Vec3.h"
#include "Vec4.h"

namespace GameEngine
{
	class WorldTransformComponent
	{
	private:
		Vec3 m_scale;
		Quaternion m_rotation;
		Vec3 m_position;
	public:
		WorldTransformComponent()
			: m_scale(1, 1, 1, CSHandedness::NONE), m_rotation(0, 0, 0, 1), m_position(0, 0, 0) { }

		void SetRotation(const Quaternion& newRotation)
		{
			m_rotation = newRotation;
		}

		Quaternion GetRotation() const
		{
			return m_rotation;
		}

		void SetScale(const Vec3& newScale)
		{
			m_scale = newScale;
		}

		Vec3 GetScale() const
		{
			return m_scale;
		}

		void SetPosition(const Vec3& newPosition)
		{
			m_position = newPosition;
		}

		Vec3 GetPosition() const
		{
			return m_position;
		}
	};
}