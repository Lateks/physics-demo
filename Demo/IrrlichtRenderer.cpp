#include "IrrlichtRenderer.h"
#include "IrrlichtRendererImpl.h"
#include "GameActor.h"
#include "WorldTransformComponent.h"
#include "GameData.h"
#include "MessagingWindow.h"
#include "Vec3.h"
#include "Mat4.h"
#include "Vec4.h"
#include <irrlicht.h>
#include <iostream>
#include <cassert>
#include <memory>

using irr::video::SColor;
using irr::u32;
using irr::core::vector3df;
using irr::core::matrix4;
using irr::core::quaternion;
using irr::video::E_DRIVER_TYPE;
using irr::video::ITexture;
using irr::scene::ISceneManager;
using irr::scene::ISceneNode;

using GameEngine::LinearAlgebra::Vec3;
using GameEngine::LinearAlgebra::Mat4;
using GameEngine::LinearAlgebra::Quaternion;

using std::weak_ptr;
using std::shared_ptr;

namespace
{
	unsigned int TEXTURE_ID = 0;
}

namespace GameEngine
{
	namespace Display
	{
		vector3df ConvertVector(Vec3& vector)
		{
			return vector3df(vector.x(), vector.y(), vector.z());
		}

		quaternion ConvertQuaternion(Quaternion& quat)
		{
			return quaternion(quat.x(), quat.y(), quat.z(), quat.w());
		}

		matrix4 ConvertProjectionMatrix(Mat4& matrix)
		{
			matrix4 newMatrix;
			for (int i = 0; i < 4; i++)
				for (int j = 0; j < 4; j++)
				{
					float value = matrix.index(i, j);
					newMatrix[i*4 + j] = (float) matrix.index(i, j);
				}
			return newMatrix;
		}

		IrrlichtRenderer::IrrlichtRenderer()
		{
			m_pData = new IrrlichtRendererImpl();
		}

		void IrrlichtRenderer::DrawScene()
		{
			UpdateActorPositions();
			assert(m_pData->m_pDriver);

			m_pData->m_pDriver->beginScene(true, true, SColor(0,0,0,0));

			m_pData->m_pSmgr->drawAll();

			m_pData->m_pDriver->clearZBuffer();
			m_pData->m_pDebugSmgr->drawAll();

			m_pData->m_pGui->drawAll();

			m_pData->m_pDriver->endScene();
		}

		// Updates all actor positions. TODO: make this event based
		// and only update the scene nodes that have actually changed
		// position, orientation etc.
		void IrrlichtRenderer::UpdateActorPositions()
		{
			GameData *game = GameData::getInstance();
			for (auto it = m_pData->sceneNodes.begin();
				it != m_pData->sceneNodes.end(); it++)
			{
				GameActor *pActor = game->GetActor(it->first);
				ISceneNode *pNode = it->second;
				
				if (pActor && pNode)
				{
					weak_ptr<WorldTransformComponent> pWeakTransform = pActor->GetWorldTransform();
					if (!pWeakTransform.expired())
					{
						shared_ptr<WorldTransformComponent> pWorldTransform(pWeakTransform);

						quaternion rot = ConvertQuaternion(pWorldTransform->GetRotation());
						vector3df eulerRot;
						rot.toEuler(eulerRot);
						eulerRot *= irr::core::RADTODEG;
						pNode->setRotation(eulerRot);

						pNode->setPosition(ConvertVector(pWorldTransform->GetPosition()));
						pNode->setScale(ConvertVector(pWorldTransform->GetScale()));
					}
				}
			}
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

		bool IrrlichtRenderer::SetupAndOpenWindow(unsigned int width, unsigned int height,
			DRIVER_TYPE driverType, CAMERA_TYPE cameraType)
		{
			m_pData->m_pDevice = irr::createDevice(GetDriverType(driverType),
				irr::core::dimension2d<u32>(width, height),
				16, false, false, false, 0);
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

		IrrlichtRenderer::~IrrlichtRenderer()
		{
			if (m_pData->m_pDevice)
			{
				m_pData->m_pDevice->drop();
			}
			delete m_pData;
		}

		void IrrlichtRenderer::SetCameraPosition(Vec3& newPosition)
		{
			assert(m_pData->m_pCamera);
			m_pData->m_pCamera->setPosition(ConvertVector(newPosition));
			m_pData->m_pCamera->updateAbsolutePosition();
		}

		void IrrlichtRenderer::SetCameraTarget(Vec3& newTarget)
		{
			assert(m_pData->m_pCamera);
			m_pData->m_pCamera->setTarget(ConvertVector(newTarget));
		}

		void IrrlichtRenderer::SetCameraProjection(Mat4& newProjection)
		{
			assert(m_pData->m_pCamera);
			m_pData->m_pCamera->setProjectionMatrix(ConvertProjectionMatrix(newProjection));
		}

		bool IrrlichtRenderer::Running()
		{
			if (!m_pData->m_pDevice)
				return false;
			return m_pData->m_pDevice->run();
		}

		bool IrrlichtRenderer::WindowActive()
		{
			if (!m_pData->m_pDevice)
				return false;
			return m_pData->m_pDevice->isWindowActive();
		}

		void IrrlichtRenderer::YieldDevice()
		{
			assert(m_pData->m_pDevice);
			m_pData->m_pDevice->yield();
		}

		unsigned int IrrlichtRenderer::LoadTexture(const std::string& filePath)
		{
			assert(m_pData->m_pDriver);
			auto texture = m_pData->m_pDriver->getTexture(filePath.c_str());
			if (!texture)
				return 0;
			m_pData->textures[++TEXTURE_ID] = texture;
			return TEXTURE_ID;
		}

		// TODO: refactor this and handle the mapping better
		void IrrlichtRenderer::AddSphereSceneNode(float radius, ActorID actorId, unsigned int texture, bool debug)
		{
			ISceneManager *manager = debug ? m_pData->m_pDebugSmgr : m_pData->m_pSmgr;
			auto node = manager->addSphereSceneNode(radius);
			ITexture *txt = m_pData->textures[texture]; // TODO: check that this key is in the map
			if (txt)
			{
				node->setMaterialTexture(0, txt);
			}
			node->setMaterialFlag(irr::video::EMF_LIGHTING, false);
			m_pData->sceneNodes[actorId] = node;
		}

		void IrrlichtRenderer::AddCubeSceneNode(float dim, ActorID actorId, unsigned int texture, bool debug)
		{
			ISceneManager *manager = debug ? m_pData->m_pDebugSmgr : m_pData->m_pSmgr;
			auto node = manager->addSphereSceneNode(dim);
			ITexture *txt = m_pData->textures[texture]; // TODO: check that this key is in the map
			if (txt)
			{
				node->setMaterialTexture(0, txt);
			}
			node->setMaterialFlag(irr::video::EMF_LIGHTING, false);
			m_pData->sceneNodes[actorId] = node;
		}

		void IrrlichtRenderer::AddMeshSceneNode(const std::string& meshFilePath, ActorID actorId, unsigned int texture, bool debug)
		{
			ISceneManager *manager = debug ? m_pData->m_pDebugSmgr : m_pData->m_pSmgr;
			auto mesh = manager->getMesh(meshFilePath.c_str());
			if (mesh)
			{
				auto node = manager->addAnimatedMeshSceneNode(mesh);
				ITexture *txt = m_pData->textures[texture]; // TODO: check that this key is in the map
				if (txt)
				{
					node->setMaterialTexture(0, txt);
				}
				node->setMaterialFlag(irr::video::EMF_LIGHTING, false);
				m_pData->sceneNodes[actorId] = node;
			}
		}

		void IrrlichtRenderer::RemoveSceneNode(ActorID actorId, bool debug)
		{
			auto node = m_pData->sceneNodes[actorId];
			if (node)
			{
				node->remove();
			}
		}
	}
}