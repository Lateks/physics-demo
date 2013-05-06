#pragma once

#include "enginefwd.h"
#include "Vec3.h"
#include "Vec4.h"
#include "IEventManager.h"

namespace GameEngine
{
	namespace Events
	{
		class BaseEventData : public IEventData
		{
		public:
			explicit BaseEventData(const float timeStamp) : m_timeStamp(timeStamp) { }
			virtual EventType VGetEventType() const = 0;
			virtual float VGetTimestamp() const override { return m_timeStamp; }
		private:
			const float m_timeStamp;
		};

		class CollisionEvent : public BaseEventData
		{
		public:
			CollisionEvent(const float timeStamp, ActorID first, ActorID second)
				: BaseEventData(timeStamp), m_collisionPair(first, second) { }
			virtual ~CollisionEvent() { };
			std::pair<ActorID, ActorID> GetCollisionPair()
			{
				return m_collisionPair;
			}
		private:
			const std::pair<ActorID, ActorID> m_collisionPair;
		};

		class ActorCollideEvent : public CollisionEvent
		{
		public:
			ActorCollideEvent(const float timeStamp, ActorID first, ActorID second,
				std::vector<Vec3> collisionManifold, Vec3 sumNormalForce, Vec3 sumFrictionForce)
				: CollisionEvent(timeStamp, first, second), m_collisionManifold(collisionManifold),
				m_sumNormalForce(sumNormalForce), m_sumFrictionForce(sumFrictionForce)
			{ }
			virtual ~ActorCollideEvent() { };
			virtual EventType VGetEventType() const override
			{
				return eventType;
			}
			const std::vector<Vec3>& GetCollisionPoints()
			{
				return m_collisionManifold;
			}
			const Vec3 GetSumNormalForce()
			{
				return m_sumNormalForce;
			}
			const Vec3 GetSumFrictionForce()
			{
				return m_sumFrictionForce;
			}
		private:
			const static EventType eventType = EventType::COLLISION_EVENT;
			std::vector<Vec3> m_collisionManifold;
			Vec3 m_sumNormalForce;
			Vec3 m_sumFrictionForce;
		};

		class ActorSeparationEvent : public CollisionEvent
		{
		public:
			ActorSeparationEvent(const float timeStamp, ActorID first, ActorID second)
				: CollisionEvent(timeStamp, first, second) { }
			virtual ~ActorSeparationEvent() { };
			virtual EventType VGetEventType() const override
			{
				return eventType;
			}
		private:
			const static EventType eventType = EventType::SEPARATION_EVENT;
		};

		class TriggerEvent : public BaseEventData
		{
		public:
			TriggerEvent(const float timeStamp, ActorID trigger, ActorID actorId)
				: BaseEventData(timeStamp), m_triggerId(trigger), m_actorId(actorId) { }
			virtual ~TriggerEvent() { };
			ActorID GetTriggerId() { return m_triggerId; }
			ActorID GetActorId() { return m_actorId; }
		private:
			ActorID m_triggerId;
			ActorID m_actorId;
		};

		class TriggerEntryEvent : public TriggerEvent
		{
		public:
			TriggerEntryEvent(const float timeStamp, ActorID trigger, ActorID actorId)
				: TriggerEvent(timeStamp, trigger, actorId) { }
			virtual ~TriggerEntryEvent() { };
			virtual EventType VGetEventType() const override
			{
				return eventType;
			}
		private:
			const static EventType eventType = EventType::ENTER_TRIGGER;
		};

		class TriggerExitEvent : public TriggerEvent
		{
		public:
			TriggerExitEvent(const float timeStamp, ActorID trigger, ActorID actorId)
				: TriggerEvent(timeStamp, trigger, actorId) { }
			virtual ~TriggerExitEvent() { };
			virtual EventType VGetEventType() const override
			{
				return eventType;
			}
		private:
			const static EventType eventType = EventType::EXIT_TRIGGER;
		};

		class ActorMoveEvent : public BaseEventData
		{
		public:
			ActorMoveEvent(const float timeStamp, ActorID actorId)
				: BaseEventData(timeStamp), m_actorId(actorId) { }
			virtual ~ActorMoveEvent() { };
			virtual EventType VGetEventType() const override
			{
				return eventType;
			}
			ActorID GetActorId()
			{
				return m_actorId;
			}
		private:
			const static EventType eventType = EventType::ACTOR_MOVED;
			ActorID m_actorId;
		};

		class CameraMoveEvent : public BaseEventData
		{
		public:
			CameraMoveEvent(const float timeStamp, Vec3 cameraPos, Vec3 cameraTarget)
				: BaseEventData(timeStamp), m_cameraPos(cameraPos), m_cameraTarget(cameraTarget) { }
			virtual ~CameraMoveEvent() { }
			virtual EventType VGetEventType() const override
			{
				return eventType;
			}
			Vec3 GetCameraPosition()
			{
				return m_cameraPos;
			}
			Vec3 GetCameraTarget()
			{
				return m_cameraTarget;
			}
		private:
			const static EventType eventType = EventType::CAMERA_MOVED;
			Vec3 m_cameraPos;
			Vec3 m_cameraTarget;
		};
	}
}