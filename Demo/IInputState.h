#pragma once

namespace GameEngine
{
	namespace Display
	{
		class IInputState
		{
		public:
			struct MouseState
			{
				MouseState() : LeftMouseDown(false), RightMouseDown(false), X(0), Y(0) { }
				MouseState(bool leftMouseDown, bool rightMouseDown, int x, int y)
					: LeftMouseDown(leftMouseDown), RightMouseDown(rightMouseDown), X(x), Y(y) { }

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