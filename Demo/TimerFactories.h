#ifndef TIMER_FACTORIES_H
#define TIMER_FACTORIES_H

#include "enginefwd.h"
#include "ITimer.h"
#include "IrrlichtRenderer.h"
#include "IrrlichtTimer.h"
#include "GameData.h"

namespace GameEngine
{
	std::auto_ptr<ITimer> GetTimer()
	{
		Display::IRenderer *renderer = GameEngine::GameData::getInstance()->GetRenderer();
		Display::IrrlichtRenderer *irrlicht = dynamic_cast<Display::IrrlichtRenderer*>(renderer);
		std::auto_ptr<ITimer> timer;
		if (irrlicht)
		{
			timer.reset(new IrrlichtTimer(irrlicht));
		}
		else
		{
			timer.reset(nullptr);
		}
		return timer;
	}
}

#endif