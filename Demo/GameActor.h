#pragma once

#include "enginefwd.h"
#include <memory>
#include <string>

namespace GameEngine
{
	struct GameActorData;

	class GameActor
	{
	public:
		GameActor(std::wstring name = L"");
		GameActor(Vec3& startPosition, std::wstring name = L"");
		virtual ~GameActor();
		ActorID GetID();
		std::wstring GetName();
		void SetWorldTransform(const WorldTransformComponent& trans);
		WorldTransformComponent& GetWorldTransform();
	private:
		std::unique_ptr<GameActorData> m_pData;
	};
}