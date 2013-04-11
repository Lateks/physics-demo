#ifndef RENDERING_ENGINE_FACTORIES_H
#define RENDERING_ENGINE_FACTORIES_H

#include "IRenderer.h"
#include "IrrlichtRenderer.h"

namespace GameEngine
{
	namespace Display
	{
		std::auto_ptr<IRenderer> CreateRenderer()
		{
			std::auto_ptr<IRenderer> renderer;
			renderer.reset(new IrrlichtRenderer());
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