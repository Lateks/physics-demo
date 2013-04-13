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

			virtual unsigned int LoadTexture(const std::string& filePath) override;

			virtual void AddSphereSceneNode(float radius, ActorID actorId, unsigned int texture, bool debug = false) override;
			virtual void AddCubeSceneNode(float dim, ActorID actorId, unsigned int texture, bool debug = false) override;
			virtual void AddMeshSceneNode(const std::string& meshFilePath, ActorID actorId, unsigned int texture = 0, bool debug = false) override;
			virtual void RemoveSceneNode(ActorID actorId, bool debug = false) override;
		private:
			IrrlichtRendererImpl *m_pData;
			void UpdateActorPositions();
		};
	}
}

#endif