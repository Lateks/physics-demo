#ifndef RENDERING_ENGINE_FACTORIES_H
#define RENDERING_ENGINE_FACTORIES_H

#include "IDisplay.h"
#include "IrrlichtDisplay.h"

namespace GameEngine
{
	namespace Display
	{
		std::unique_ptr<IDisplay> CreateRenderer()
		{
			std::unique_ptr<IDisplay> renderer;
			renderer.reset(new IrrlichtDisplay());
			if (!renderer.get())
			{
				renderer.reset();
				return renderer;
			}
			return renderer;
		}
	}
}

#endif