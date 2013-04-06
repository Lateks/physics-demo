#ifndef BASIC_IRRLICHT_RENDERER_IMPL_H
#define BASIC_IRRLICHT_RENDERER_IMPL_H
namespace irr
{
	namespace video
	{
		class IVideoDriver;
	}
	namespace scene
	{
		class ISceneManager;
	}
	namespace gui
	{
		class IGUIEnvironment;
	}
	class IrrlichtDevice;
}

class MessagingWindow;

struct BasicIrrlichtRendererImpl
{
	BasicIrrlichtRendererImpl() { }
	~BasicIrrlichtRendererImpl();

	irr::IrrlichtDevice *device;
	irr::video::IVideoDriver *driver;
	irr::scene::ISceneManager *scene;
	irr::gui::IGUIEnvironment *gui;
	float lastMessage;
	MessagingWindow *messages;

	void Update();
};

#endif