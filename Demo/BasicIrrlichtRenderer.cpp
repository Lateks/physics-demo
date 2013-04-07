#include "stdafx.h"
#include "BasicIrrlichtRenderer.h"
#include "BasicIrrlichtRendererImpl.h"
#include "MessagingWindow.h"
#include <irrlicht.h>
#include <iostream>

using irr::IrrlichtDevice;
using irr::createDevice;
using irr::u32;
using irr::video::SColor;

BasicIrrlichtRenderer::BasicIrrlichtRenderer()
{
	pImpl = new BasicIrrlichtRendererImpl();
}

BasicIrrlichtRenderer::~BasicIrrlichtRenderer()
{
	delete pImpl;
}

bool BasicIrrlichtRenderer::SetupAndOpenWindow(unsigned int width, unsigned int height, irr::video::E_DRIVER_TYPE driverType)
{
	pImpl->device =
		createDevice(driverType, irr::core::dimension2d<u32>(width, height),
		16, false, false, false, 0);
	if (pImpl->device == nullptr)
	{
		return false;
	}
	pImpl->device->setWindowCaption(L"3D world demo");

	pImpl->driver = pImpl->device->getVideoDriver();
	pImpl->scene = pImpl->device->getSceneManager();
	pImpl->gui = pImpl->device->getGUIEnvironment();
	pImpl->debugSmgr = pImpl->scene->createNewSceneManager(false);

	irr::scene::ISceneNode *cube = pImpl->scene->addCubeSceneNode();
	if (cube)
	{
		cube->setMaterialTexture(0, pImpl->driver->getTexture("..\\texture\\woodbox2.jpg"));
		cube->setMaterialFlag(irr::video::EMF_LIGHTING, false);
		irr::scene::ISceneNodeAnimator *anim = pImpl->scene
			->createFlyStraightAnimator(irr::core::vector3df(-25,0,60),
			                            irr::core::vector3df(25,0,60),
										3000, true, true);
		if (anim)
		{
			cube->addAnimator(anim);
			anim->drop();
		}
	}
	irr::scene::ISceneNode *sphere = pImpl->scene->addSphereSceneNode();
	if (sphere)
	{
		sphere->setMaterialTexture(0, pImpl->driver->getTexture("..\\texture\\cracked_mud.JPG"));
		sphere->setMaterialFlag(irr::video::EMF_LIGHTING, false);
		irr::scene::ISceneNodeAnimator *anim = pImpl->scene
			->createFlyStraightAnimator(irr::core::vector3df(25,0,40),
			                            irr::core::vector3df(-25,0,80),
										2000, true, true);
		if (anim)
		{
			sphere->addAnimator(anim);
			anim->drop();
		}
	}
	irr::scene::IAnimatedMesh *arrow = pImpl->debugSmgr->addArrowMesh("arrowMesh", SColor(255, 255, 0, 0), SColor(255, 255, 0, 0), 4u, 8u, 20.0f, 19.0f, 0.05f, 0.5f);
	irr::scene::ISceneNode *arrowNode = pImpl->debugSmgr->addAnimatedMeshSceneNode(arrow);
	if (arrowNode)
	{
		arrowNode->setPosition(irr::core::vector3df(0,0,60));
		arrowNode->setRotation(irr::core::vector3df(270,0,0));
		arrowNode->setMaterialFlag(irr::video::EMF_LIGHTING, false);
		arrowNode->setVisible(true);
	}
	irr::scene::ICameraSceneNode *pCamera = pImpl->scene->addCameraSceneNodeFPS();
	if (pCamera)
	{
		pCamera->setPosition(irr::core::vector3df(0,100,0));
		pCamera->setTarget(irr::core::vector3df(0,0,0));
	}
	pImpl->debugSmgr->setActiveCamera(pCamera);
	irr::gui::IGUIFont *font = pImpl->gui->getFont("..\\font\\fontlucida.png");
	pImpl->messages = new MessagingWindow(200, 150);
	pImpl->messages->SetPosition(10, 10);
	pImpl->messages->SetFont(font);

	return true;
}

bool BasicIrrlichtRenderer::IsRunning()
{
	return pImpl->device->run();
}

void BasicIrrlichtRenderer::UpdateScene()
{
	if (pImpl->device->isWindowActive())
	{
		pImpl->Update();
	}
	else
		pImpl->device->yield();
}