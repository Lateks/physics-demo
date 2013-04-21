#include "IrrlichtDisplayImpl.h"
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
			pNode->setMaterialFlag(irr::video::EMF_LIGHTING, false);

			weak_ptr<WorldTransformComponent> pWeakTransform = pStrongActor->GetWorldTransform();
			if (!pWeakTransform.expired())
			{
				shared_ptr<WorldTransformComponent> pWorldTransform(pWeakTransform);
				SetNodeTransform(pNode, pWorldTransform);
			}

			sceneNodes[pStrongActor->GetID()] = pNode;
			auto game = GameData::getInstance();
			game->GetEventManager()->RegisterHandler(EventType::ACTOR_MOVED,
				m_pMoveEventHandler);
		}

		void IrrlichtDisplayImpl::UpdateActorPosition(EventPtr pEvent)
		{
			assert(pEvent->GetEventType() == EventType::ACTOR_MOVED);
			ActorMoveEvent *pMoveEvent =
				dynamic_cast<ActorMoveEvent*>(pEvent.get());

			GameData *game = GameData::getInstance();
			WeakActorPtr pWeakActor = game->GetActor(pMoveEvent->GetActorId());
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

			pNode->setPosition(ConvertVectorWithHandedness(pWorldTransform->GetPosition()));
			pNode->setScale(ConvertVector(pWorldTransform->GetScale()));
		}

		vector3df IrrlichtDisplayImpl::ConvertVectorWithHandedness(Vec3& vector)
		{
			return vector3df(vector.x(), vector.y(), -vector.z());
		}

		Vec3 IrrlichtDisplayImpl::ConvertVectorWithHandedness(vector3df& vector)
		{
			return Vec3(vector.X, vector.Y, -vector.Z);
		}

		vector3df IrrlichtDisplayImpl::ConvertVector(Vec3& vector)
		{
			return vector3df(vector.x(), vector.y(), vector.z());
		}

		Vec3 IrrlichtDisplayImpl::ConvertVector(vector3df& vector)
		{
			return Vec3(vector.X, vector.Y, vector.Z);
		}

		quaternion IrrlichtDisplayImpl::ConvertQuaternion(Quaternion& quat)
		{
			return quaternion(quat.x(), quat.y(), quat.z(), quat.w());
		}

		// TODO: handedness
		matrix4 IrrlichtDisplayImpl::ConvertProjectionMatrix(Mat4& matrix)
		{
			matrix4 newMatrix;
			for (int i = 0; i < 4; i++)
				for (int j = 0; j < 4; j++)
				{
					float value = matrix.index(i, j);
					newMatrix[i*4 + j] = (float) matrix.index(i, j);
				}
			return newMatrix;
		}
	}
}