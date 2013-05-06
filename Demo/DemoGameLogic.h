#pragma once

#include "IGameLogic.h"

namespace GameEngine
{
	struct DemoGameLogicData;

	class DemoGameLogic : public IGameLogic
	{
	public:
		DemoGameLogic();
		virtual ~DemoGameLogic();
		virtual void VHandleInputs() override;
		virtual void VSetupInitialScene() override;
	private:
		std::unique_ptr<DemoGameLogicData> m_pData;
	};
}