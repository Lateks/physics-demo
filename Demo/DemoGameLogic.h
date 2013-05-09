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
}