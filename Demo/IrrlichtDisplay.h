#ifndef IRRLICHT_DISPLAY_H
#define IRRLICHT_DISPLAY_H

#include "enginefwd.h"
#include "IDisplay.h"
#include "IEventManager.h"
#include <memory>

namespace GameEngine
{
	namespace Display
	{
		class IrrlichtDisplay : public IDisplay
		{
		public:
			friend class IrrlichtTimer;

			IrrlichtDisplay();
			~IrrlichtDisplay();

			virtual std::shared_ptr<IInputState> GetInputState() const override;

			virtual void YieldDevice() override;
			virtual bool Running() override;
			virtual bool WindowActive() override;
			virtual bool SetupAndOpenWindow(unsigned int width, unsigned int height,
				DRIVER_TYPE driverType, CAMERA_TYPE cameraType) override;
			virtual void DrawScene() override;

			virtual void SetCameraPosition(Vec3& newPosition) override;
			virtual void SetCameraTarget(Vec3& newTarget) override;
			virtual void SetCameraProjection(Mat4& newProjection) override;
			virtual Vec3 GetCameraPosition() const override;
			virtual Vec3 GetCameraTarget() const override;
			virtual Vec3 GetCameraUpVector() const override;
			virtual Vec3 GetCameraRightVector() const override;

			virtual unsigned int LoadTexture(const std::string& filePath) override;

			virtual void AddSphereSceneNode(float radius, WeakActorPtr pActor, unsigned int texture, bool debug = false) override;
			virtual void AddCubeSceneNode(float dim, WeakActorPtr pActor, unsigned int texture, bool debug = false) override;
			virtual void AddMeshSceneNode(const std::string& meshFilePath, WeakActorPtr pActor, unsigned int texture = 0, bool debug = false) override;
			virtual void RemoveSceneNode(ActorID actorId, bool debug = false) override;

			virtual void LoadMap(const std::string& mapFilePath, const std::string& meshName, Vec3& position) override;
		private:
			IrrlichtDisplayImpl *m_pData;
		};
	}
}

#endif