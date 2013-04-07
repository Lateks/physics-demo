#include "stdafx.h"
#include "BasicIrrlichtRendererImpl.h"
#include "MessagingWindow.h"
#include <irrlicht.h>

using irr::video::SColor;

void BasicIrrlichtRendererImpl::Update()
{
	irr::u32 currentFrame = device->getTimer()->getTime();
	if (lastMessage != 0)
	{
		double deltaTime = (currentFrame - lastMessage) / 1000.0;
		if (deltaTime >= 1.0)
		{
			irr::core::stringw message = L"Current time: ";
			message += deltaTime;
			messages->AddMessage(message);
			lastMessage = currentFrame;
		}
	}
	else
	{
		lastMessage = currentFrame;
	}
	driver->beginScene(true, true, SColor(0,0,0,0));

	scene->drawAll();
	driver->clearZBuffer();
	debugSmgr->drawAll();

	gui->drawAll();

	messages->Render();

	driver->endScene();
}

BasicIrrlichtRendererImpl::~BasicIrrlichtRendererImpl()
{
	if (device != nullptr)
		device->drop();
}