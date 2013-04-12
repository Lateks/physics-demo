#include "IrrlichtRenderer.h"
#include "IrrlichtRendererImpl.h"
#include "MessagingWindow.h"
#include "Vec3.h"
#include "Mat4.h"
#include <irrlicht.h>
#include <iostream>
#include <cassert>

using irr::video::SColor;
using irr::u32;
using irr::core::vector3df;
using irr::core::matrix4;
using irr::video::E_DRIVER_TYPE;

using GameEngine::LinearAlgebra::Vec3;
using GameEngine::LinearAlgebra::Mat4;

namespace
{
	unsigned int TEXTURE_ID = 0;
}

namespace GameEngine
{
	namespace Display
	{
		IrrlichtRenderer::IrrlichtRenderer()
		{
			m_pData = new IrrlichtRendererImpl();
		}

		void IrrlichtRenderer::DrawScene()
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

		vector3df ConvertVector(Vec3& vector)
		{
			return vector3df((float) vector.getX(), (float) vector.getY(), (float) vector.getZ());
		}

		matrix4 ConvertMatrix(Mat4& matrix)
		{
			matrix4 newMatrix;
			for (int i = 0; i < 4; i++)
				for (int j = 0; j < 4; j++)
					newMatrix[i*4 + j] = (float) matrix.index(i, j);
			return newMatrix;
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
			m_pData->m_pCamera->setProjectionMatrix(ConvertMatrix(newProjection));
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
	}
}