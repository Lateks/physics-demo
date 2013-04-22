#include "IrrlichtDisplay.h"
#include "IrrlichtDisplayImpl.h"
#include "IrrlichtInputState.h"
#include "IrrlichtConversions.h"
#include "GameActor.h"
#include "WorldTransformComponent.h"
#include "GameData.h"
#include "EventManager.h"
#include "Events.h"
#include "MessagingWindow.h"
#include "Vec3.h"
#include "Mat4.h"
#include "Vec4.h"
#include <irrlicht.h>
#include <iostream>
#include <cassert>
#include <memory>

using irr::u32;

using irr::video::SColor;
using irr::video::E_DRIVER_TYPE;
using irr::video::ITexture;

using irr::core::vector3df;
using irr::core::matrix4;
using irr::core::quaternion;

using irr::scene::ISceneManager;
using irr::scene::ISceneNode;
using irr::scene::IAnimatedMesh;

using std::weak_ptr;
using std::shared_ptr;

using GameEngine::Events::IEventData;
using GameEngine::Events::EventType;
using GameEngine::Events::EventPtr;
using GameEngine::Events::ActorMoveEvent;

namespace
{
	unsigned int TEXTURE_ID = 0;
}

namespace GameEngine
{
	namespace Display
	{
		IrrlichtDisplay::IrrlichtDisplay()
		{
			m_pData = new IrrlichtDisplayImpl();
			m_pData->m_pInputState.reset(new IrrlichtInputState());
			m_pData->m_pMoveEventHandler =
				Events::EventHandlerPtr(new std::function<void(EventPtr)>(
				[this] (EventPtr event) { this->m_pData->UpdateActorPosition(event); }));
		}

		Vec3 IrrlichtDisplay::GetCameraPosition() const
		{
			vector3df pos = m_pData->m_pCamera->getAbsolutePosition();
			return ConvertVector(pos);
		}

		std::shared_ptr<IInputState> IrrlichtDisplay::GetInputState() const
		{
			return m_pData->m_pInputState;
		}

		void IrrlichtDisplay::DrawScene()
		{
			assert(m_pData->m_pDriver);

			m_pData->m_pDriver->beginScene(true, true, SColor(0,0,0,0));

			m_pData->m_pSmgr->drawAll();

			m_pData->m_pDriver->clearZBuffer();
			m_pData->m_pDebugSmgr->drawAll();

			m_pData->m_pGui->drawAll();

			m_pData->m_pDriver->endScene();
		}

		E_DRIVER_TYPE GetDriverType(DRIVER_TYPE type)
		{
			switch (type)
			{
			case DRIVER_TYPE::DIRECT_3D8:
				return E_DRIVER_TYPE::EDT_DIRECT3D8;
			case DRIVER_TYPE::DIRECT_3D9:
				return E_DRIVER_TYPE::EDT_DIRECT3D9;
			case DRIVER_TYPE::OPEN_GL:
				return E_DRIVER_TYPE::EDT_OPENGL;
			case DRIVER_TYPE::SOFTWARE:
				return E_DRIVER_TYPE::EDT_SOFTWARE;
			default:
				return E_DRIVER_TYPE::EDT_SOFTWARE;
			}
		}

		bool IrrlichtDisplay::SetupAndOpenWindow(unsigned int width, unsigned int height,
			DRIVER_TYPE driverType, CAMERA_TYPE cameraType)
		{
			m_pData->m_pDevice = irr::createDevice(GetDriverType(driverType),
				irr::core::dimension2d<u32>(width, height),
				16, false, false, false, m_pData->m_pInputState.get());
			if (!m_pData->m_pDevice)
			{
				return false;
			}

			m_pData->m_pDriver = m_pData->m_pDevice->getVideoDriver();
			m_pData->m_pSmgr = m_pData->m_pDevice->getSceneManager();
			m_pData->m_pGui = m_pData->m_pDevice->getGUIEnvironment();
			m_pData->m_pDebugSmgr = m_pData->m_pSmgr->createNewSceneManager(false);

			irr::gui::IGUIFont *font = m_pData->m_pGui->getFont("..\\assets\\fontlucida.png");

			switch (cameraType)
			{
			case CAMERA_TYPE::STATIC:
				m_pData->m_pCamera = m_pData->m_pSmgr->addCameraSceneNode();
				break;
			default:
				m_pData->m_pCamera = m_pData->m_pSmgr->addCameraSceneNodeFPS();
			}
			m_pData->m_pDebugSmgr->setActiveCamera(m_pData->m_pCamera);
			m_pData->m_pDevice->getCursorControl()->setVisible(false);

			return true;
		}

		IrrlichtDisplay::~IrrlichtDisplay()
		{
			if (m_pData->m_pDevice)
			{
				m_pData->m_pDevice->drop();
			}
			delete m_pData;
		}

		void IrrlichtDisplay::SetCameraPosition(Vec3& newPosition)
		{
			assert(m_pData->m_pCamera);
			m_pData->m_pCamera->setPosition(ConvertVector(newPosition));
			m_pData->m_pCamera->updateAbsolutePosition();
		}

		void IrrlichtDisplay::SetCameraTarget(Vec3& newTarget)
		{
			assert(m_pData->m_pCamera);
			m_pData->m_pCamera->setTarget(ConvertVector(newTarget));
		}

		void IrrlichtDisplay::SetCameraProjection(Mat4& newProjection)
		{
			assert(m_pData->m_pCamera);
			m_pData->m_pCamera->setProjectionMatrix(ConvertProjectionMatrix(newProjection));
		}

		bool IrrlichtDisplay::Running()
		{
			if (!m_pData->m_pDevice)
				return false;
			return m_pData->m_pDevice->run();
		}

		bool IrrlichtDisplay::WindowActive()
		{
			if (!m_pData->m_pDevice)
				return false;
			return m_pData->m_pDevice->isWindowActive();
		}

		void IrrlichtDisplay::YieldDevice()
		{
			assert(m_pData->m_pDevice);
			m_pData->m_pDevice->yield();
		}

		unsigned int IrrlichtDisplay::LoadTexture(const std::string& filePath)
		{
			assert(m_pData->m_pDriver);
			auto texture = m_pData->m_pDriver->getTexture(filePath.c_str());
			if (!texture)
				return 0;
			m_pData->textures[++TEXTURE_ID] = texture;
			return TEXTURE_ID;
		}

		void IrrlichtDisplay::AddSphereSceneNode(float radius, WeakActorPtr pActor, unsigned int texture, bool debug)
		{
			ISceneManager *manager = debug ? m_pData->m_pDebugSmgr : m_pData->m_pSmgr;
			auto node = manager->addSphereSceneNode(radius);
			m_pData->AddSceneNode(pActor, node, texture);
		}

		void IrrlichtDisplay::AddCubeSceneNode(float dim, WeakActorPtr pActor, unsigned int texture, bool debug)
		{
			ISceneManager *manager = debug ? m_pData->m_pDebugSmgr : m_pData->m_pSmgr;
			auto node = manager->addCubeSceneNode(dim);
			m_pData->AddSceneNode(pActor, node, texture);
		}

		void IrrlichtDisplay::AddMeshSceneNode(const std::string& meshFilePath, WeakActorPtr pActor, unsigned int texture, bool debug)
		{
			ISceneManager *manager = debug ? m_pData->m_pDebugSmgr : m_pData->m_pSmgr;
			auto mesh = manager->getMesh(meshFilePath.c_str());
			if (mesh)
			{
				auto node = manager->addAnimatedMeshSceneNode(mesh);
				m_pData->AddSceneNode(pActor, node, texture);
			}
		}

		void IrrlichtDisplay::RemoveSceneNode(ActorID actorId, bool debug)
		{
			auto node = m_pData->sceneNodes[actorId];
			if (node)
			{
				node->remove();
			}
		}

		void IrrlichtDisplay::LoadMap(const std::string& mapFilePath,
			const std::string& meshName, Vec3& position)
		{
			if (!m_pData->m_pDevice || !m_pData->m_pSmgr)
				return;
			m_pData->m_pDevice->getFileSystem()->addFileArchive(mapFilePath.c_str());

			ISceneManager *pSmgr = m_pData->m_pSmgr;
			IAnimatedMesh *mesh = pSmgr->getMesh(meshName.c_str());
			ISceneNode *node = nullptr;

			if (mesh)
			{
				node = pSmgr->addOctreeSceneNode(mesh->getMesh(0), 0, -1, 1024);
			}
			if (node)
			{
				node->setPosition(ConvertVector(position));
			}
		}

		Vec3 IrrlichtDisplay::GetCameraTarget() const
		{
			auto cameraLookAt = m_pData->m_pCamera->getTarget();
			return ConvertVector(cameraLookAt);
		}

		Vec3 IrrlichtDisplay::GetCameraUpVector() const
		{
			auto cameraUp = m_pData->m_pCamera->getUpVector();
			return ConvertVector(cameraUp);
		}

		Vec3 IrrlichtDisplay::GetCameraRightVector() const
		{
			auto camera = m_pData->m_pCamera;
			auto up = camera->getUpVector();
			auto forward = camera->getTarget() - camera->getAbsolutePosition();
			auto rightVector = up.crossProduct(forward);

			return ConvertVector(rightVector);
		}

		Quaternion IrrlichtDisplay::GetCameraRotation() const
		{
			vector3df cameraRotEuler = m_pData->m_pCamera->getRotation();
			quaternion quaternionRot(cameraRotEuler * irr::core::DEGTORAD);
			return ConvertQuaternion(quaternionRot);
		}
	}
}