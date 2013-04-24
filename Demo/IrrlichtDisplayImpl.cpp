#include "IrrlichtDisplayImpl.h"
#include "IrrlichtConversions.h"
#include "IEventManager.h"
#include "Events.h"
#include "GameData.h"
#include "GameActor.h"
#include "WorldTransformComponent.h"
#include "Vec4.h"
#include <irrlicht.h>
#include <map>
#include <memory>
#include <cassert>

using irr::scene::ISceneNode;
using irr::video::ITexture;

using irr::core::vector3df;
using irr::core::matrix4;
using irr::core::quaternion;

using std::weak_ptr;
using std::shared_ptr;

using GameEngine::Events::IEventData;
using GameEngine::Events::EventType;
using GameEngine::Events::EventPtr;
using GameEngine::Events::ActorMoveEvent;

namespace GameEngine
{
	namespace Display
	{
		ISceneNode *IrrlichtDisplayImpl::GetSceneNode(ActorID actorId)
		{
			auto it = sceneNodes.find(actorId);
			assert(it != sceneNodes.end());
			if (it != sceneNodes.end())
				return it->second;
			return nullptr;
		}

		void IrrlichtDisplayImpl::AddSceneNode(WeakActorPtr pActor, irr::scene::ISceneNode *pNode, unsigned int texture)
		{
			if (pActor.expired())
			{
				pNode->drop();
				return;
			}

			StrongActorPtr pStrongActor(pActor);

			auto texturePos = textures.find(texture);
			if (texturePos != textures.end())
			{
				pNode->setMaterialTexture(0, (*texturePos).second);
			}
			else
			{
				pNode->setMaterialFlag(irr::video::EMF_WIREFRAME, true);
				pNode->setMaterialFlag(irr::video::EMF_BACK_FACE_CULLING, false);
			}
			pNode->setMaterialFlag(irr::video::EMF_LIGHTING, false);

			weak_ptr<WorldTransformComponent> pWeakTransform = pStrongActor->GetWorldTransform();
			if (!pWeakTransform.expired())
			{
				shared_ptr<WorldTransformComponent> pWorldTransform(pWeakTransform);
				SetNodeTransform(pNode, pWorldTransform);
			}

			sceneNodes[pStrongActor->GetID()] = pNode;
			auto game = GameData::GetInstance();
			game->GetEventManager()->RegisterHandler(EventType::ACTOR_MOVED,
				m_pMoveEventHandler);
		}

		void IrrlichtDisplayImpl::UpdateActorPosition(EventPtr pEvent)
		{
			assert(pEvent->GetEventType() == EventType::ACTOR_MOVED);
			ActorMoveEvent *pMoveEvent =
				dynamic_cast<ActorMoveEvent*>(pEvent.get());

			std::shared_ptr<GameData> pGame = GameData::GetInstance();
			WeakActorPtr pWeakActor = pGame->GetActor(pMoveEvent->GetActorId());
			if (pWeakActor.expired())
				return;

			StrongActorPtr pActor(pWeakActor);
			ISceneNode *pNode = GetSceneNode(pActor->GetID());

			if (pActor.get() && pNode)
			{
				weak_ptr<WorldTransformComponent> pWeakTransform = pActor->GetWorldTransform();
				if (!pWeakTransform.expired())
				{
					shared_ptr<WorldTransformComponent> pWorldTransform(pWeakTransform);
					SetNodeTransform(pNode, pWorldTransform);
				}
			}
		}

		void IrrlichtDisplayImpl::SetNodeTransform(
			irr::scene::ISceneNode *pNode, std::shared_ptr<WorldTransformComponent> pWorldTransform)
		{
			quaternion rot = ConvertQuaternion(pWorldTransform->GetRotation());
			vector3df eulerRot;
			rot.toEuler(eulerRot);
			eulerRot *= irr::core::RADTODEG;
			pNode->setRotation(eulerRot);

			pNode->setPosition(ConvertVector(pWorldTransform->GetPosition()));
			pNode->setScale(ConvertVector(pWorldTransform->GetScale()));
		}
	}
}