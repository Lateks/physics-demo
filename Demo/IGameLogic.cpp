#include "IGameLogic.h"
#include "DemoGameLogic.h"

namespace GameEngine
{
	std::unique_ptr<IGameLogic> CreateDemoGameLogic()
	{
		return std::unique_ptr<IGameLogic>(new DemoGameLogic());
	}
}