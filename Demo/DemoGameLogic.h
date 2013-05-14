#pragma once

#include "IGameLogic.h"

namespace Demo
{
	struct DemoGameLogicData;

	class DemoGameLogic : public GameEngine::IGameLogic
	{
	public:
		DemoGameLogic();
		virtual ~DemoGameLogic();
		virtual void VUpdate(float deltaSec) override;
		virtual bool VSetupInitialScene() override;
	private:
		std::unique_ptr<DemoGameLogicData> m_pData;
	};

	class DemoGameLogicFactory : public GameEngine::IGameLogicFactory
	{
	public:
		virtual ~DemoGameLogicFactory() { }

		virtual std::shared_ptr<GameEngine::IGameLogic> CreateGameLogic() const
		{
			auto pLogic = std::make_shared<DemoGameLogic>();
			if (pLogic && !pLogic->VSetupInitialScene())
			{
				pLogic.reset();
			}
			return pLogic;
		}
	};
}