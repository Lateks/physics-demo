#ifndef I_RENDERER_H
#define I_RENDERER_H

#include "enginefwd.h"
#include <string>

namespace GameEngine
{
	namespace Display
	{
		enum DRIVER_TYPE
		{
			OPEN_GL,
			DIRECT_3D8,
			DIRECT_3D9,
			SOFTWARE
		};

		enum CAMERA_TYPE
		{
			FPS,
			STATIC
		};

		class IRenderer
		{
		public:
			virtual ~IRenderer() { }

			virtual void YieldDevice() = 0;
			virtual bool Running() = 0;
			virtual bool WindowActive() = 0;
			virtual bool SetupAndOpenWindow(unsigned int width, unsigned int height,
				DRIVER_TYPE driverType, CAMERA_TYPE cameraType) = 0;
			virtual void DrawScene() = 0;

			virtual void SetCameraPosition(LinearAlgebra::Vec3& newPosition) = 0;
			virtual void SetCameraTarget(LinearAlgebra::Vec3& newTarget) = 0;
			virtual void SetCameraProjection(LinearAlgebra::Mat4& newProjection) = 0;
		};
	}
}

#endif