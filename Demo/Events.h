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
			virtual const EventType& VGetEventType(void) const = 0;
			float GetTimeStamp(void) const { return m_timeStamp; }
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
			virtual EventType GetEventType() const override;
			std::pair<ActorID, ActorID> GetCollisionPair()
			{
				return m_collisionPair;
			}
		private:
			const static EventType m_eventType;
			const std::pair<ActorID, ActorID> m_collisionPair;
		};

		class SeparationEvent : public CollisionEvent
		{
		public:
			SeparationEvent(const float timeStamp, ActorID first, ActorID second)
				: CollisionEvent(timeStamp, first, second) { }
			virtual ~SeparationEvent() { };
			virtual EventType GetEventType() const override;
		private:
			const static EventType m_eventType;
		};

		class ActorMoveEvent : public BaseEventData
		{
		public:
			ActorMoveEvent(const float timeStamp, ActorID actorId)
				: BaseEventData(timeStamp), m_actorId(actorId) { }
			virtual ~ActorMoveEvent() { };
			ActorID GetActorId()
			{
				return m_actorId;
			}
		private:
			const static EventType m_eventType;
			ActorID m_actorId;
		};
	}
}

#endif