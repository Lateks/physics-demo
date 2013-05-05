#ifndef EVENTS_H
#define EVENTS_H

#include "enginefwd.h"
#include "Vec3.h"
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

		class RayChangeEvent : public BaseEventData
		{
		public:
			RayChangeEvent(const float timeStamp, Vec3 rayFrom, Vec3 rayTo)
				: BaseEventData(timeStamp), m_rayFrom(rayFrom), m_rayTo(rayTo) { }
			virtual ~RayChangeEvent() { }
			Vec3 GetRayFrom()
			{
				return m_rayFrom;
			}
			Vec3 GetRayTo()
			{
				return m_rayTo;
			}
		private:
			Vec3 m_rayFrom;
			Vec3 m_rayTo;
		};

		class CameraMoveEvent : public RayChangeEvent
		{
		public:
			CameraMoveEvent(const float timeStamp, Vec3 cameraPos, Vec3 cameraTarget)
				: RayChangeEvent(timeStamp, cameraPos, cameraTarget) { }
			virtual ~CameraMoveEvent() { }
			virtual EventType VGetEventType() const override
			{
				return eventType;
			}
		private:
			const static EventType eventType = EventType::CAMERA_MOVED;
		};
	}
}

#endif