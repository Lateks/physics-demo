#ifndef DEMO_INPUT_HANDLER_H
#define DEMO_INPUT_HANDLER_H

#include "enginefwd.h"
#include "IGameInputHandler.h"
#include "IInputState.h"

namespace GameEngine
{
	class DemoInputHandler : public IGameInputHandler
	{
	public:
		virtual ~DemoInputHandler() { };
		virtual void HandleInputs() override;
		virtual void SetupInitialScene(GameData *game) override;
	private:
		Display::IInputState::MouseState m_previousMouseState;
		Display::IInputState::MouseState m_currentMouseState;
		ActorID m_pickedActor;

		bool LeftMousePressed();
		bool LeftMouseDown();
		bool LeftMouseReleased();
		bool RightMousePressed();
		bool RightMouseDown();
		bool RightMouseReleased();

		void ThrowCube(Vec3& throwTowards);
	};
}

#endif