#include "IrrlichtTimer.h"
#include "IrrlichtDisplay.h"
#include "IrrlichtDisplayImpl.h"

using GameEngine::Display::IrrlichtDisplay;

namespace GameEngine
{
	IrrlichtTimer::~IrrlichtTimer() { }

	unsigned int IrrlichtTimer::GetTimeMs()
	{
		return m_pRenderer->m_pData->m_pDevice->getTimer()->getTime();
	}
}