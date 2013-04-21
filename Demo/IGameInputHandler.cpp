#include "IGameInputHandler.h"
#include "DemoInputHandler.h"

namespace GameEngine
{
	std::unique_ptr<IGameInputHandler> CreateDemoInputHandler()
	{
		std::unique_ptr<IGameInputHandler> handler(new DemoInputHandler());
		if (!handler.get())
		{
			handler.reset(nullptr);
		}
		return handler;
	}
}