#ifndef IRRLICHT_RENDERER_IMPL_H
#define IRRLICHT_RENDERER_IMPL_H

#include "enginefwd.h"
#include <irrlicht.h>
#include <map>

namespace GameEngine
{
	namespace Display
	{
		struct IrrlichtRendererImpl
		{
			std::map<unsigned int, irr::video::ITexture*> textures;
			std::map<ActorID, irr::scene::ISceneNode*> sceneNodes;

			irr::IrrlichtDevice *m_pDevice;
			irr::video::IVideoDriver *m_pDriver;
			irr::scene::ISceneManager *m_pSmgr;

			// The second scene manager is used for handling "debug"
			// objects (e.g. collision meshes, collision normals etc.).
			// This scene is drawn on top.
			irr::scene::ISceneManager *m_pDebugSmgr;

			irr::scene::ICameraSceneNode *m_pCamera;
			irr::gui::IGUIEnvironment *m_pGui;
		};
	}
}

#endif