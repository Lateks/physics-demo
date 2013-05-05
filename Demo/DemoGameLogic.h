#ifndef DEMO_INPUT_HANDLER_H
#define DEMO_INPUT_HANDLER_H

#include "enginefwd.h"
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

#endif