#ifndef EVENTS_H
#define EVENTS_H

#include "enginefwd.h"
#include "IEventManager.h"

namespace GameEngine
{
	namespace Events
	{
		class BaseEventData : public IEventData
		{
		public:
			explicit BaseEventData(const float timeStamp) : m_timeStamp(timeStamp) { }
			virtual EventType GetEventType() const = 0;
			float GetTimestamp() const { return m_timeStamp; }
		private:
			const float m_timeStamp;
		};

		// TODO: what other data does this require?
		// Collision normals or something?
		class CollisionEvent : public BaseEventData
		{
		public:
			CollisionEvent(const float timeStamp, ActorID first, ActorID second)
				: BaseEventData(timeStamp), m_collisionPair(first, second) { }
			virtual ~CollisionEvent() { };
			virtual EventType GetEventType() const override
			{
				return eventType;
			}
			std::pair<ActorID, ActorID> GetCollisionPair()
			{
				return m_collisionPair;
			}
		private:
			const static EventType eventType = EventType::COLLISION_EVENT;
			const std::pair<ActorID, ActorID> m_collisionPair;
		};

		class SeparationEvent : public CollisionEvent
		{
		public:
			SeparationEvent(const float timeStamp, ActorID first, ActorID second)
				: CollisionEvent(timeStamp, first, second) { }
			virtual ~SeparationEvent() { };
			virtual EventType GetEventType() const override
			{
				return eventType;
			}
		private:
			const static EventType eventType = EventType::SEPARATION_EVENT;
		};

		class ActorMoveEvent : public BaseEventData
		{
		public:
			ActorMoveEvent(const float timeStamp, ActorID actorId)
				: BaseEventData(timeStamp), m_actorId(actorId) { }
			virtual ~ActorMoveEvent() { };
			virtual EventType GetEventType() const override
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
	}
}

#endif