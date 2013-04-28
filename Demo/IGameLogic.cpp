#include "IGameLogic.h"
#include "DemoGameLogic.h"

namespace GameEngine
{
	std::unique_ptr<IGameLogic> CreateDemoGameLogic()
	{
		std::unique_ptr<IGameLogic> handler(new DemoGameLogic());
		if (!handler.get())
		{
			handler.reset(nullptr);
		}
		return handler;
	}
}