#include "IrrlichtRenderer.h"
#include "MessagingWindow.h"
#include "Vec3.h"
#include "Mat4.h"
#include <irrlicht.h>
#include <iostream>

using irr::video::SColor;
using irr::u32;
using irr::core::vector3df;
using irr::core::matrix4;
using irr::video::E_DRIVER_TYPE;

using GameEngine::LinearAlgebra::Vec3;
using GameEngine::LinearAlgebra::Mat4;

namespace GameEngine
{
	namespace Display
	{
		void IrrlichtRenderer::DrawScene()
		{
			m_pDriver->beginScene(true, true, SColor(0,0,0,0));

			m_pSmgr->drawAll();

			m_pDriver->clearZBuffer();
			m_pDebugSmgr->drawAll();

			m_pGui->drawAll();

			m_pDriver->endScene();
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
			m_pDevice = irr::createDevice(GetDriverType(driverType),
				irr::core::dimension2d<u32>(width, height),
				16, false, false, false, 0);
			if (!m_pDevice)
			{
				return false;
			}

			m_pDriver = m_pDevice->getVideoDriver();
			m_pSmgr = m_pDevice->getSceneManager();
			m_pGui = m_pDevice->getGUIEnvironment();
			m_pDebugSmgr = m_pSmgr->createNewSceneManager(false);

			irr::gui::IGUIFont *font = m_pGui->getFont("..\\assets\\fontlucida.png");

			switch (cameraType)
			{
			case CAMERA_TYPE::STATIC:
				m_pCamera = m_pSmgr->addCameraSceneNode();
				break;
			default:
				m_pCamera = m_pSmgr->addCameraSceneNodeFPS();
			}
			m_pDebugSmgr->setActiveCamera(m_pCamera);
			m_pDevice->getCursorControl()->setVisible(false);

			return true;
		}

		IrrlichtRenderer::~IrrlichtRenderer()
		{
			m_pDevice->drop();
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
			m_pCamera->setPosition(ConvertVector(newPosition));
			m_pCamera->updateAbsolutePosition();
		}

		void IrrlichtRenderer::SetCameraTarget(Vec3& newTarget)
		{
			m_pCamera->setTarget(ConvertVector(newTarget));
		}

		void IrrlichtRenderer::SetCameraProjection(Mat4& newProjection)
		{
			m_pCamera->setProjectionMatrix(ConvertMatrix(newProjection));
		}

		bool IrrlichtRenderer::Running()
		{
			return m_pDevice->run();
		}

		bool IrrlichtRenderer::WindowActive()
		{
			return m_pDevice->isWindowActive();
		}

		void IrrlichtRenderer::YieldDevice()
		{
			m_pDevice->yield();
		}
	}
}