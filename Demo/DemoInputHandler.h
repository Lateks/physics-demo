#ifndef DEMO_INPUT_HANDLER_H
#define DEMO_INPUT_HANDLER_H

#include "enginefwd.h"
#include "IGameInputHandler.h"
#include "IInputState.h"
#include "Vec3.h"

namespace GameEngine
{
	class DemoInputHandler : public IGameInputHandler
	{
	public:
		virtual ~DemoInputHandler() { };
		virtual void HandleInputs() override;
		virtual void SetupInitialScene(GameData *game) override;
	private:
		struct CameraState
		{
			CameraState() { }
			CameraState(Vec3& pos, Vec3& target)
				: cameraPos(pos), cameraTarget(target) { };
			Vec3 cameraPos;
			Vec3 cameraTarget;
		};

		Display::IInputState::MouseState m_previousMouseState;
		Display::IInputState::MouseState m_currentMouseState;
		CameraState m_previousCameraState;
		CameraState m_currentCameraState;

		ActorID m_pickedActor;
		unsigned int m_pickConstraintId;

		bool CameraMoved();
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