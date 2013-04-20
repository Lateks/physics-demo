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
			DIRECT_3D8,
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

			virtual std::shared_ptr<IInputState> GetInputState() const = 0;

			virtual void YieldDevice() = 0;
			virtual bool Running() = 0;
			virtual bool WindowActive() = 0;
			virtual bool SetupAndOpenWindow(unsigned int width, unsigned int height,
				DRIVER_TYPE driverType, CAMERA_TYPE cameraType) = 0;
			virtual void DrawScene() = 0;

			virtual void SetCameraPosition(Vec3& newPosition) = 0;
			virtual void SetCameraTarget(Vec3& newTarget) = 0;
			virtual void SetCameraProjection(Mat4& newProjection) = 0;

			// Returns a unique id that can be used to refer to the texture.
			// Returns 0 if loading fails.
			virtual unsigned int LoadTexture(const std::string& filePath) = 0;

			virtual void AddSphereSceneNode(float radius, ActorID actorId, unsigned int texture, bool debug = false) = 0;
			virtual void AddCubeSceneNode(float dim, ActorID actorId, unsigned int texture, bool debug = false) = 0;
			virtual void AddMeshSceneNode(const std::string& meshFilePath, ActorID actorId, unsigned int texture = 0, bool debug = false) = 0;
			virtual void RemoveSceneNode(ActorID actorId, bool debug = false) = 0;

			virtual void LoadMap(const std::string& mapFilePath, const std::string& meshName, Vec3& position) = 0;
		};
	}
}

#endif