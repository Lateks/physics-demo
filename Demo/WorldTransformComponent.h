#pragma once

#include "enginefwd.h"

namespace GameEngine
{
	struct WorldTransformComponentData;

	class WorldTransformComponent
	{
	private:
		std::unique_ptr<WorldTransformComponentData> m_pData;
	public:
		WorldTransformComponent();
		WorldTransformComponent(const WorldTransformComponent& other);
		WorldTransformComponent(WorldTransformComponent&& other);
		virtual ~WorldTransformComponent();

		WorldTransformComponent& operator=(const WorldTransformComponent& other);
		WorldTransformComponent& operator=(WorldTransformComponent&& other);

		void SetRotation(const Quaternion& newRotation);
		Quaternion GetRotation() const;

		void SetScale(const Vec3& newScale);
		Vec3 GetScale() const;

		void SetPosition(const Vec3& newPosition);
		Vec3 GetPosition() const;
	};
}