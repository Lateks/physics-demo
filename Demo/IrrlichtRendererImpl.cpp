#include "IrrlichtRendererImpl.h"
#include <irrlicht.h>
#include <map>
#include <cassert>

using irr::scene::ISceneNode;

namespace GameEngine
{
	namespace Display
	{
		ISceneNode *IrrlichtRendererImpl::GetSceneNode(ActorID actorId)
		{
			auto it = sceneNodes.find(actorId);
			assert(it != sceneNodes.end());
			if (it != sceneNodes.end())
				return it->second;
			return nullptr;
		}
	}
}