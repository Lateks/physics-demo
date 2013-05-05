#ifndef IRRLICHT_DISPLAY_H
#define IRRLICHT_DISPLAY_H

#include "enginefwd.h"
#include "IDisplay.h"
#include "ITimer.h"
#include "IEventManager.h"
#include <memory>

namespace GameEngine
{
	namespace Display
	{
		struct IrrlichtDisplayData;
		class IrrlichtTimer;

		class IrrlichtDisplay : public IDisplay
		{
		public:
			friend class IrrlichtTimer;

			IrrlichtDisplay();
			~IrrlichtDisplay();

			virtual std::shared_ptr<IInputState> VGetInputState() const override;

			virtual void VYieldDevice() override;
			virtual bool VRunning() override;
			virtual bool VWindowActive() override;
			virtual bool VSetupAndOpenWindow(unsigned int width, unsigned int height,
				DRIVER_TYPE driverType, CAMERA_TYPE cameraType) override;
			virtual void VDrawScene() override;

			virtual std::shared_ptr<MessagingWindow> VGetMessageWindow() override;

			virtual void VSetCameraPosition(Vec3& newPosition) override;
			virtual void VSetCameraTarget(Vec3& newTarget) override;
			virtual void VSetCameraFOV(double degrees) override;
			virtual void VSetCameraNearPlaneDistance(double distance) override;
			virtual void VSetCameraFarPlaneDistance(double distance) override;

			virtual Vec3 VGetCameraPosition() const override;
			virtual Vec3 VGetCameraTarget() const override;
			virtual Vec3 VGetCameraUpVector() const override;
			virtual Vec3 VGetCameraRightVector() const override;
			virtual Quaternion VGetCameraRotation() const override;

			virtual void VHideCursor() override;
			virtual void VShowCursor() override;

			virtual unsigned int VLoadTexture(const std::string& filePath) override;

			virtual void VAddSphereSceneNode(float radius, WeakActorPtr pActor, unsigned int texture) override;
			virtual void VAddCubeSceneNode(float dim, WeakActorPtr pActor, unsigned int texture) override;
			virtual void VAddMeshSceneNode(const std::string& meshFilePath, WeakActorPtr pActor, unsigned int texture = 0) override;
			virtual void VRemoveSceneNode(ActorID actorId) override;

			virtual void VLoadMap(const std::string& mapFilePath, const std::string& meshName, Vec3& position) override;
		private:
			IrrlichtDisplay(IrrlichtDisplay& other);
			IrrlichtDisplay& operator=(IrrlichtDisplay& other);
			std::unique_ptr<IrrlichtDisplayData> m_pData;
		};

		class IrrlichtTimer : public ITimer
		{
		public:
			IrrlichtTimer(std::shared_ptr<IrrlichtDisplay> pDisplay);
			virtual unsigned int GetTimeMs() override;
		private:
			std::weak_ptr<IrrlichtDisplay> m_pDisplay;
		};
	}
}

#endif