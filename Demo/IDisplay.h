#pragma once

#include "enginefwd.h"
#include "Vec4.h"
#include <string>

namespace GameEngine
{
	namespace Display
	{
		enum class DriverType : uint8_t
		{
			NO_WINDOW,
			OPEN_GL,
			DIRECT_3D9,
			SOFTWARE
		};

		enum class CameraType : uint8_t
		{
			FPS,
			FPS_WASD,
			STATIC
		};

		class IDisplay
		{
		public:
			virtual ~IDisplay() { }

			virtual unsigned int VGetDeviceTimeMs() const = 0;

			virtual std::shared_ptr<IInputState> VGetInputState() const = 0;

			virtual bool VSetupAndOpenWindow(unsigned int width, unsigned int height,
				DriverType driverType, CameraType cameraType) = 0;

			virtual void VYieldDevice() = 0;
			virtual bool VRunning() = 0;
			virtual bool VWindowActive() = 0;
			virtual void VDrawScene() = 0;

			virtual std::shared_ptr<MessagingWindow> VGetMessageWindow() = 0;

			virtual void VSetCameraPosition(Vec3& newPosition) = 0;
			virtual void VSetCameraTarget(Vec3& newTarget) = 0;
			virtual void VSetCameraUpVector(Vec3& newUpVector) = 0;
			virtual void VSetCameraRotation(Quaternion newRotation) = 0;
			virtual void VSetCameraFOV(float degrees) = 0;
			virtual void VSetCameraNearPlaneDistance(float distance) = 0;
			virtual void VSetCameraFarPlaneDistance(float distance) = 0;

			virtual Vec3 VGetCameraPosition() const = 0;
			virtual Vec3 VGetCameraTarget() const = 0;
			virtual Vec3 VGetCameraUpVector() const = 0;
			virtual Vec3 VGetCameraRightVector() const = 0;
			virtual Quaternion VGetCameraRotation() const = 0;
			virtual float VGetCameraFOV() const = 0;
			virtual float VGetCameraNearPlaneDistance() const = 0;
			virtual float VGetCameraFarPlaneDistance() const = 0;

			virtual void VHideCursor() = 0;
			virtual void VShowCursor() = 0;
			virtual bool VCursorVisible() const = 0;

			// Returns a unique id that can be used to refer to the texture.
			// Returns 0 if loading fails.
			virtual unsigned int VLoadTexture(const std::string& filePath) = 0;
			virtual void VLoadMap(const std::string& mapFilePath, const std::string& meshName, Vec3& position) = 0;

			virtual void VAddSphereSceneNode(float radius, ActorPtr pActor, unsigned int texture = 0, bool lightingOn = false) = 0;
			virtual void VAddCubeSceneNode(float dim, ActorPtr pActor, unsigned int texture = 0, bool lightingOn = false) = 0;
			virtual void VAddMeshSceneNode(const std::string& meshFilePath, ActorPtr pActor, unsigned int texture = 0, bool lightingOn = false) = 0;
			virtual void VAddLightSceneNode(const Vec3& position, const RGBAColor& color, float lightRadius) = 0;

			virtual void VRemoveSceneNode(ActorID actorId) = 0;

			virtual void VSetSceneNodeLighting(ActorID actorId, bool lightingOn) = 0;
			virtual void VSetSceneNodeLightColors(ActorID actorId, const RGBAColor& specularColor = RGBAColor::White,
				const RGBAColor& ambientColor = RGBAColor::White, const RGBAColor& diffuseColor = RGBAColor::White) = 0;
			virtual void VSetSceneNodeShininess(ActorID actorId, float shininess) = 0;

			virtual void VSetGlobalAmbientLight(const RGBAColor& color) = 0;
		};

		class IDisplayFactory
		{
			virtual std::shared_ptr<IDisplay> VCreateDeviceAndOpenWindow() = 0;
		};
	}
}