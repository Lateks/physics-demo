#include "WorldTransformComponent.h"
#include "Vec3.h"
#include "Vec4.h"

namespace GameEngine
{
	struct WorldTransformComponentData
	{
		WorldTransformComponentData()
			: m_scale(1, 1, 1, CSHandedness::NONE), m_rotation(0, 0, 0, 1), m_position(0, 0, 0) { }
		Vec3 m_scale;
		Quaternion m_rotation;
		Vec3 m_position;
	};

	WorldTransformComponent::WorldTransformComponent()
		: m_pData(new WorldTransformComponentData()) { }

	WorldTransformComponent::WorldTransformComponent(const WorldTransformComponent& other)
		: m_pData(new WorldTransformComponentData())
	{
		m_pData->m_scale = other.m_pData->m_scale;
		m_pData->m_rotation = other.m_pData->m_rotation;
		m_pData->m_position = other.m_pData->m_position;
	}

	WorldTransformComponent::WorldTransformComponent(WorldTransformComponent&& other)
	{
		m_pData = std::move(other.m_pData);
	}

	WorldTransformComponent& WorldTransformComponent::operator=(const WorldTransformComponent& other)
	{
		if (this != &other)
		{
			m_pData->m_scale = other.m_pData->m_scale;
			m_pData->m_rotation = other.m_pData->m_rotation;
			m_pData->m_position = other.m_pData->m_position;
		}
		return *this;
	}

	WorldTransformComponent& WorldTransformComponent::operator=(WorldTransformComponent&& other)
	{
		if (this != &other)
		{
			m_pData = std::move(other.m_pData);
		}
		return *this;
	}

	WorldTransformComponent::~WorldTransformComponent() { }

	void WorldTransformComponent::SetRotation(const Quaternion& newRotation)
	{
		m_pData->m_rotation = newRotation;
	}

	Quaternion WorldTransformComponent::GetRotation() const
	{
		return m_pData->m_rotation;
	}

	void WorldTransformComponent::SetScale(const Vec3& newScale)
	{
		m_pData->m_scale = newScale;
	}

	Vec3 WorldTransformComponent::GetScale() const
	{
		return m_pData->m_scale;
	}

	void WorldTransformComponent::SetPosition(const Vec3& newPosition)
	{
		m_pData->m_position = newPosition;
	}

	Vec3 WorldTransformComponent::GetPosition() const
	{
		return m_pData->m_position;
	}
}