#pragma once

#include "enginefwd.h"
#include "IDisplay.h"
#include "ITimer.h"
#include <memory>

namespace GameEngine
{
	namespace Display
	{
		class IrrlichtDisplayFactory : public IDisplayFactory
		{
		public:
			virtual std::shared_ptr<IDisplay> VSetupAndOpenWindow(unsigned int width,
				unsigned int height, DriverType driverType, CameraType cameraType) override;
		};

		struct IrrlichtDisplayData;
		class IrrlichtTimer;

		class IrrlichtDisplay : public IDisplay
		{
		public:
			friend class IrrlichtTimer;
			friend class IrrlichtDisplayFactory;

			~IrrlichtDisplay();

			virtual std::shared_ptr<IInputState> VGetInputState() const override;

			virtual void VYieldDevice() override;
			virtual bool VRunning() override;
			virtual bool VWindowActive() override;
			virtual void VDrawScene() override;

			virtual std::shared_ptr<MessagingWindow> VGetMessageWindow() override;

			virtual void VSetCameraPosition(Vec3& newPosition) override;
			virtual void VSetCameraTarget(Vec3& newTarget) override;
			virtual void VSetCameraUpVector(Vec3& newUpVector) override;
			virtual void VSetCameraRotation(Quaternion newRotation) override;
			virtual void VSetCameraFOV(float degrees) override;
			virtual void VSetCameraNearPlaneDistance(float distance) override;
			virtual void VSetCameraFarPlaneDistance(float distance) override;

			virtual Vec3 VGetCameraPosition() const override;
			virtual Vec3 VGetCameraTarget() const override;
			virtual Vec3 VGetCameraUpVector() const override;
			virtual Vec3 VGetCameraRightVector() const override;
			virtual Quaternion VGetCameraRotation() const override;
			virtual float VGetCameraFOV() const override;
			virtual float VGetCameraNearPlaneDistance() const override;
			virtual float VGetCameraFarPlaneDistance() const override;

			virtual void VHideCursor() override;
			virtual void VShowCursor() override;
			virtual bool VCursorVisible() const override;

			virtual unsigned int VLoadTexture(const std::string& filePath) override;
			virtual void VLoadMap(const std::string& mapFilePath, const std::string& meshName, Vec3& position) override;

			// If no texture id is given as a parameter (the texture parameter is 0),
			// the scene node is rendered as a wireframe).
			virtual void VAddSphereSceneNode(float radius, ActorPtr pActor, unsigned int texture, bool lightingOn) override;
			virtual void VAddCubeSceneNode(float dim, ActorPtr pActor, unsigned int texture, bool lightingOn) override;
			virtual void VAddMeshSceneNode(const std::string& meshFilePath, ActorPtr pActor, unsigned int texture, bool lightingOn) override;
			virtual void VAddLightSceneNode(const Vec3& position, const RGBAColor& color, float lightRadius) override;

			virtual void VRemoveSceneNode(ActorID actorId) override;
			virtual void VSetSceneNodeLighting(ActorID actorId, bool lightingOn) override;
			virtual void VSetGlobalAmbientLight(const RGBAColor& color) override;
			virtual void VSetSceneNodeLightColors(ActorID actorId, const RGBAColor& specularColor,
				const RGBAColor& ambientColor, const RGBAColor& diffuseColor) override;
			virtual void VSetSceneNodeShininess(ActorID actorId, float shininess) override;
		private:
			IrrlichtDisplay();
			IrrlichtDisplay(IrrlichtDisplay& other);
			IrrlichtDisplay& operator=(IrrlichtDisplay& other);
			bool IrrlichtDisplay::VSetupAndOpenWindow(unsigned int width, unsigned int height,
				DriverType driverType, CameraType cameraType);
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