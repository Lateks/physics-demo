#ifndef WORLD_TRANSFORM_COMPONENT_H
#define WORLD_TRANSFORM_COMPONENT_H

#include "enginefwd.h"
#include "Mat4.h"
#include "Vec3.h"
#include "Vec4.h"

namespace GameEngine
{
	class WorldTransformComponent
	{
	private:
		LinearAlgebra::Vec3 m_scale;
		LinearAlgebra::Quaternion m_rotation;
		LinearAlgebra::Vec3 m_position;
	public:
		WorldTransformComponent()
			: m_scale(1, 1, 1), m_rotation(0, 0, 0, 0), m_position(0, 0, 0) { }

		void SetRotation(const LinearAlgebra::Quaternion& newRotation)
		{
			m_rotation = newRotation;
		}
		LinearAlgebra::Quaternion GetRotation() const
		{
			return m_rotation;
		}

		void SetScale(const LinearAlgebra::Vec3& newScale)
		{
			m_scale = newScale;
		}
		LinearAlgebra::Vec3 GetScale() const
		{
			return m_scale;
		}

		void SetPosition(const LinearAlgebra::Vec3& newPosition)
		{
			m_position = newPosition;
		}
		LinearAlgebra::Vec3 GetPosition() const
		{
			return m_position;
		}
	};
}

#endif