#ifndef BASIC_IRRLICHT_RENDERER_IMPL_H
#define BASIC_IRRLICHT_RENDERER_IMPL_H

#include <irrlicht.h>

class MessagingWindow;

// This struct holds all the components required to render
// scenes with Irrlicht as well as some helper methods.
struct IrrlichtRenderer
{
	IrrlichtRenderer() { }
	~IrrlichtRenderer();

	irr::IrrlichtDevice *pDevice;
	irr::video::IVideoDriver *pDriver;
	irr::scene::ISceneManager *pSmgr;

	// The second scene manager is used for handling "debug"
	// objects (e.g. collision meshes, collision normals etc.).
	// This scene is drawn on top.
	irr::scene::ISceneManager *pDebugSmgr;

	irr::scene::ICameraSceneNode *pCamera;
	irr::gui::IGUIEnvironment *pGui;
	MessagingWindow *pMessages;

	bool SetupAndOpenWindow(unsigned int width, unsigned int height, irr::video::E_DRIVER_TYPE driverType, bool fpsCamera = true);
	void DrawScene(bool debug = false, bool drawAxes = false);
};

#endif