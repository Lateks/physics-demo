#ifndef IRRLICHT_TIMER_H
#define IRRLICHT_TIMER_H

#include "enginefwd.h"
#include "ITimer.h"

namespace GameEngine
{
	class IrrlichtTimer : public ITimer
	{
	public:
		IrrlichtTimer(Display::IrrlichtRenderer *renderer)
			: m_pRenderer(renderer) { }
		~IrrlichtTimer();
		virtual unsigned int GetTime() override;
	private:
		Display::IrrlichtRenderer *m_pRenderer;
	};
}

#endif