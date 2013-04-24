#ifndef I_GAME_INPUT_HANDLER_H
#define I_GAME_INPUT_HANDLER_H

#include "enginefwd.h"
#include <memory>

namespace GameEngine
{
	class IGameInputHandler
	{
	public:
		virtual ~IGameInputHandler() { };
		virtual void HandleInputs() = 0;
		virtual void SetupInitialScene() = 0;
	};

	std::unique_ptr<IGameInputHandler> CreateDemoInputHandler();
}

#endif