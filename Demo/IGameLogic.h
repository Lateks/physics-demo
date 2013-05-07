#ifndef I_GAME_INPUT_HANDLER_H
#define I_GAME_INPUT_HANDLER_H

#include "enginefwd.h"
#include <memory>

namespace GameEngine
{
	class IGameLogic
	{
	public:
		virtual ~IGameLogic() { };
		virtual void VUpdate(float deltaSec) = 0;
		virtual bool VSetupInitialScene() = 0;
	};

	std::unique_ptr<IGameLogic> CreateDemoGameLogic();
}

#endif