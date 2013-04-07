#ifndef BASIC_IRRLICHT_RENDERER_IMPL_H
#define BASIC_IRRLICHT_RENDERER_IMPL_H

#include <irrlicht.h>

class MessagingWindow;

struct BasicIrrlichtRendererImpl
{
	BasicIrrlichtRendererImpl() { }
	~BasicIrrlichtRendererImpl();

	irr::IrrlichtDevice *device;
	irr::video::IVideoDriver *driver;
	irr::scene::ISceneManager *scene;
	irr::scene::ISceneManager *debugSmgr;
	irr::gui::IGUIEnvironment *gui;
	irr::u32 lastMessage;
	MessagingWindow *messages;

	void Update();
};

#endif