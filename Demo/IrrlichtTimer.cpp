#include "IrrlichtTimer.h"
#include "IrrlichtRenderer.h"
#include "IrrlichtRendererImpl.h"

using GameEngine::Display::IrrlichtRenderer;

namespace GameEngine
{
	IrrlichtTimer::~IrrlichtTimer() { }

	unsigned int IrrlichtTimer::GetTime()
	{
		return m_pRenderer->m_pData->m_pDevice->getTimer()->getTime();
	}
}