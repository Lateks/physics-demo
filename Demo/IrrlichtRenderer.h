#ifndef BASIC_IRRLICHT_RENDERER_H
#define BASIC_IRRLICHT_RENDERER_H

#include "enginefwd.h"
#include "IRenderer.h"

namespace GameEngine
{
	namespace Display
	{
		class IrrlichtRenderer : public IRenderer
		{
		public:
			friend class IrrlichtTimer;

			IrrlichtRenderer();
			~IrrlichtRenderer();

			virtual void YieldDevice() override;
			virtual bool Running() override;
			virtual bool WindowActive() override;
			virtual bool SetupAndOpenWindow(unsigned int width, unsigned int height,
				DRIVER_TYPE driverType, CAMERA_TYPE cameraType) override;
			virtual void DrawScene() override;

			virtual void SetCameraPosition(LinearAlgebra::Vec3& newPosition) override;
			virtual void SetCameraTarget(LinearAlgebra::Vec3& newTarget) override;
			virtual void SetCameraProjection(LinearAlgebra::Mat4& newProjection) override;
		private:
			IrrlichtRendererImpl *m_pData;
		};
	}
}

#endif