#ifndef I_INPUT_STATE_H
#define I_INPUT_STATE_H

namespace GameEngine
{
	namespace Display
	{
		class IInputState
		{
		public:
			struct MouseState
			{
				bool LeftMouseDown;
				bool RightMouseDown;
				int X;
				int Y;
			};

			virtual ~IInputState() { }

			const MouseState GetMouseState() const
			{
				return m_mouseState;
			}
		protected:
			MouseState m_mouseState;
		};
	}
}

#endif