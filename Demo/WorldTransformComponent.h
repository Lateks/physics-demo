#ifndef WORLD_TRANSFORM_COMPONENT_H
#define WORLD_TRANSFORM_COMPONENT_H

#include "enginefwd.h"
#include "Mat4.h"
#include "Vec3.h"

namespace GameEngine
{
	class WorldTransformComponent
	{
	private:
		LinearAlgebra::Mat4 m_transform;
	public:
		void SetTransform(const LinearAlgebra::Mat4& newTransform)
		{
			m_transform = newTransform;
		}
		LinearAlgebra::Mat4 GetTransform() const
		{
			return m_transform;
		}
		void SetPosition(const LinearAlgebra::Vec3& newPosition)
		{
			m_transform.index(0, 3) = newPosition.getX();
			m_transform.index(1, 3) = newPosition.getY();
			m_transform.index(2, 3) = newPosition.getZ();
		}
		LinearAlgebra::Vec3 GetPosition() const
		{
			return LinearAlgebra::Vec3(m_transform.index(0, 3),
				m_transform.index(1, 3), m_transform.index(2, 3));
		}
	};
}

#endif