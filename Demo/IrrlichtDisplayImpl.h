#ifndef IRRLICHT_DISPLAY_IMPL_H
#define IRRLICHT_DISPLAY_IMPL_H

#include "enginefwd.h"
#include "IEventManager.h"
#include <irrlicht.h>
#include <map>
#include <vector>
#include <memory>

namespace GameEngine
{
	namespace Display
	{
		struct IrrlichtDisplayImpl
		{
			std::map<unsigned int, irr::video::ITexture*> textures;
			std::map<ActorID, irr::scene::ISceneNode*> sceneNodes;

			irr::scene::ISceneNode *GetSceneNode(ActorID actorId);

			irr::IrrlichtDevice *m_pDevice;
			irr::video::IVideoDriver *m_pDriver;
			irr::scene::ISceneManager *m_pSmgr;

			// The second scene manager is used for handling "debug"
			// objects (e.g. collision meshes, collision normals etc.).
			// This scene is drawn on top.
			irr::scene::ISceneManager *m_pDebugSmgr;

			irr::scene::ICameraSceneNode *m_pCamera;
			irr::gui::IGUIEnvironment *m_pGui;
			Events::EventHandlerPtr m_pMoveEventHandler;

			std::shared_ptr<IrrlichtInputState> m_pInputState;
			std::shared_ptr<MessagingWindow> m_messageWindow;

			void AddSceneNode(WeakActorPtr pActor, irr::scene::ISceneNode *pNode, unsigned int texture);
			void UpdateActorPosition(Events::EventPtr pEvent);
		private:
			void SetNodeTransform(irr::scene::ISceneNode *pNode, std::shared_ptr<WorldTransformComponent> pWorldTransform);
		};
	}
}

#endif