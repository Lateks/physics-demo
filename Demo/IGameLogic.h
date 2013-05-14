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

	class IGameLogicFactory
	{
	public:
		virtual ~IGameLogicFactory() { }
		virtual std::shared_ptr<IGameLogic> CreateGameLogic() = 0;
	};
}

#endif