#include "IrrlichtTimer.h"
#include "IrrlichtRenderer.h"

using GameEngine::Display::IrrlichtRenderer;

namespace GameEngine
{
	IrrlichtTimer::~IrrlichtTimer() { }

	unsigned int IrrlichtTimer::GetTime()
	{
		return m_pRenderer->m_pDevice->getTimer()->getTime();
	}
}