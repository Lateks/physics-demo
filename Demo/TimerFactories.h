#ifndef TIMER_FACTORIES_H
#define TIMER_FACTORIES_H

#include "enginefwd.h"
#include "ITimer.h"
#include "IrrlichtDisplay.h"
#include "GameData.h"

namespace GameEngine
{
	std::unique_ptr<ITimer> GetTimer()
	{
		auto pDisplay = GameEngine::GameData::GetInstance()->GetDisplayComponent();
		std::shared_ptr<Display::IrrlichtDisplay> pIrrlicht =
			std::dynamic_pointer_cast<Display::IrrlichtDisplay>(pDisplay);
		std::unique_ptr<ITimer> timer;
		if (pIrrlicht)
		{
			timer.reset(new Display::IrrlichtTimer(pIrrlicht));
		}
		return timer;
	}
}

#endif