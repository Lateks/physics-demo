#include "stdafx.h"
#include "BasicIrrlichtRendererImpl.h"
#include "MessagingWindow.h"
#include <irrlicht.h>

using irr::video::SColor;

void BasicIrrlichtRendererImpl::Update()
{
	float currentFrame = device->getTimer()->getTime();
	if (lastMessage != 0)
	{
		float deltaTime = (currentFrame - lastMessage) / 1000.0;
		if (deltaTime >= 1.0f)
		{
			irr::core::stringw message = L"Current time: ";
			message += currentFrame;
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
	gui->drawAll();

	messages->Render();

	driver->endScene();
}

BasicIrrlichtRendererImpl::~BasicIrrlichtRendererImpl()
{
	if (device != nullptr)
		device->drop();
}