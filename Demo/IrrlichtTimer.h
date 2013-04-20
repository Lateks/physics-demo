#ifndef IRRLICHT_TIMER_H
#define IRRLICHT_TIMER_H

#include "enginefwd.h"
#include "ITimer.h"

namespace GameEngine
{
	class IrrlichtTimer : public ITimer
	{
	public:
		IrrlichtTimer(Display::IrrlichtDisplay *renderer)
			: m_pRenderer(renderer) { }
		~IrrlichtTimer();
		virtual unsigned int GetTimeMs() override;
	private:
		Display::IrrlichtDisplay *m_pRenderer;
	};
}

#endif