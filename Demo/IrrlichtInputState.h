#ifndef IRRLICHT_INPUT_STATE_H
#define IRRLICHT_INPUT_STATE_H

#include "IInputState.h"
#include <irrlicht.h>

namespace GameEngine
{
	namespace Display
	{
		class IrrlichtInputState : public IInputState, public irr::IEventReceiver
		{
		public:
			virtual ~IrrlichtInputState() { }

			virtual bool OnEvent(const irr::SEvent& event)
			{
				if (event.EventType == irr::EET_MOUSE_INPUT_EVENT)
				{
					m_mouseState.LeftMouseDown = event.MouseInput.isLeftPressed();
					m_mouseState.RightMouseDown = event.MouseInput.isRightPressed();
					m_mouseState.X = event.MouseInput.X;
					m_mouseState.Y = event.MouseInput.Y;
				}
				return false;
			}
		};
	}
}

#endif