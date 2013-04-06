#include "stdafx.h"
#include "BasicIrrlichtRendererImpl.h"
#include <irrlicht.h>

using irr::video::SColor;

void BasicIrrlichtRendererImpl::Update()
{
	driver->beginScene(true, true, SColor(0,0,0,0));
	scene->drawAll();
	gui->drawAll();
	driver->endScene();
}

BasicIrrlichtRendererImpl::~BasicIrrlichtRendererImpl()
{
	if (device != nullptr)
		device->drop();
}