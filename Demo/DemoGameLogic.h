#ifndef DEMO_INPUT_HANDLER_H
#define DEMO_INPUT_HANDLER_H

#include "enginefwd.h"
#include "IGameLogic.h"
#include "IInputState.h"
#include "IEventManager.h"
#include "Vec3.h"

namespace GameEngine
{
	class DemoGameLogic : public IGameLogic
	{
	public:
		virtual ~DemoGameLogic() { };
		virtual void HandleInputs() override;
		virtual void SetupInitialScene() override;
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

		void PrintTriggerEvent(std::shared_ptr<Display::MessagingWindow> pMessages, Events::EventPtr event);
		void ThrowCube(Vec3& throwTowards);
	};
}

#endif