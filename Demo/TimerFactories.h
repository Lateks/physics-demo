#ifndef TIMER_FACTORIES_H
#define TIMER_FACTORIES_H

#include "enginefwd.h"
#include "ITimer.h"
#include "IrrlichtDisplay.h"
#include "IrrlichtTimer.h"
#include "GameData.h"

namespace GameEngine
{
	std::unique_ptr<ITimer> GetTimer()
	{
		Display::IDisplay *renderer = GameEngine::GameData::getInstance()->GetRenderer();
		Display::IrrlichtDisplay *irrlicht = dynamic_cast<Display::IrrlichtDisplay*>(renderer);
		std::unique_ptr<ITimer> timer;
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