#ifndef I_DISPLAY_H
#define I_DISPLAY_H

#include "enginefwd.h"
#include <string>

namespace GameEngine
{
	namespace Display
	{
		enum class DRIVER_TYPE
		{
			OPEN_GL,
			DIRECT_3D9,
			SOFTWARE
		};

		enum class CAMERA_TYPE
		{
			FPS,
			STATIC
		};

		class IDisplay
		{
		public:
			virtual ~IDisplay() { }

			virtual std::shared_ptr<IInputState> VGetInputState() const = 0;

			virtual void VYieldDevice() = 0;
			virtual bool VRunning() = 0;
			virtual bool VWindowActive() = 0;
			virtual bool VSetupAndOpenWindow(unsigned int width, unsigned int height,
				DRIVER_TYPE driverType, CAMERA_TYPE cameraType) = 0;
			virtual void VDrawScene() = 0;

			virtual std::shared_ptr<MessagingWindow> VGetMessageWindow() = 0;

			virtual void VSetCameraPosition(Vec3& newPosition) = 0;
			virtual void VSetCameraTarget(Vec3& newTarget) = 0;

			virtual Vec3 VGetCameraPosition() const = 0;
			virtual Vec3 VGetCameraTarget() const = 0;
			virtual Vec3 VGetCameraUpVector() const = 0;
			virtual Vec3 VGetCameraRightVector() const = 0;
			virtual Quaternion VGetCameraRotation() const = 0;

			// Returns a unique id that can be used to refer to the texture.
			// Returns 0 if loading fails.
			virtual unsigned int VLoadTexture(const std::string& filePath) = 0;

			virtual void VAddSphereSceneNode(float radius, WeakActorPtr pActor, unsigned int texture) = 0;
			virtual void VAddCubeSceneNode(float dim, WeakActorPtr pActor, unsigned int texture) = 0;
			virtual void VAddMeshSceneNode(const std::string& meshFilePath, WeakActorPtr pActor, unsigned int texture = 0) = 0;
			virtual void VRemoveSceneNode(ActorID actorId) = 0;

			virtual void VLoadMap(const std::string& mapFilePath, const std::string& meshName, Vec3& position) = 0;
		};
	}
}

#endif