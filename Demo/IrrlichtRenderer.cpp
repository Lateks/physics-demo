#include "stdafx.h"
#include "IrrlichtRenderer.h"
#include "MessagingWindow.h"
#include <irrlicht.h>

using irr::video::SColor;
using irr::u32;

void IrrlichtRenderer::DrawScene(bool debug)
{
	pDriver->beginScene(true, true, SColor(0,0,0,0));

	pSmgr->drawAll();

	if (debug)
	{
		pDriver->clearZBuffer();
		pDebugSmgr->drawAll();
	}

	pGui->drawAll();

	pMessages->Render();

	pDriver->endScene();
}

bool IrrlichtRenderer::SetupAndOpenWindow(unsigned int width, unsigned int height,
										  irr::video::E_DRIVER_TYPE driverType, bool fpsCamera)
{
	pDevice = irr::createDevice(driverType, irr::core::dimension2d<u32>(width, height),
		16, false, false, false, 0);
	if (pDevice == nullptr)
	{
		return false;
	}

	pDevice->setWindowCaption(L"3D world demo");

	pDriver = pDevice->getVideoDriver();
	pSmgr = pDevice->getSceneManager();
	pGui = pDevice->getGUIEnvironment();
	pDebugSmgr = pSmgr->createNewSceneManager(false);

	irr::gui::IGUIFont *font = pGui->getFont("..\\font\\fontlucida.png");
	pMessages = new MessagingWindow(200, 150);
	pMessages->SetPosition(10, 10);
	pMessages->SetFont(font);

	if (fpsCamera)
	{
		pCamera = pSmgr->addCameraSceneNodeFPS();
	}
	else
	{
		pCamera = pSmgr->addCameraSceneNode();
	}
	pDebugSmgr->setActiveCamera(pCamera);
	pDevice->getCursorControl()->setVisible(false);

	return true;
}

IrrlichtRenderer::~IrrlichtRenderer()
{
	if (pDevice != nullptr)
		pDevice->drop();
}