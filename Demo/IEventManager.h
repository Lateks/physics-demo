#ifndef I_EVENT_MANAGER_H
#define I_EVENT_MANAGER_H

#include <functional>
#include <memory>

namespace GameEngine
{
	namespace Events
	{
		// Just some event types needed for communication between
		// the physics system and other parts of the game engine.
		// Would need to be extended in an actual, more fleshed
		// out game engine.
		enum class EventType
		{
			COLLISION_EVENT,
			SEPARATION_EVENT,
			ACTOR_MOVED,
			ENTER_TRIGGER,
			EXIT_TRIGGER
		};

		class IEventData
		{
		public:
			virtual ~IEventData() { };
			virtual EventType GetEventType() const = 0;
			virtual float GetTimestamp() const = 0;
		};

		typedef std::shared_ptr<IEventData> EventPtr;
		typedef std::function<void(EventPtr)> EventHandler;
		typedef std::shared_ptr<EventHandler> EventHandlerPtr;

		class IEventManager
		{
		public:
			IEventManager() { }
			virtual ~IEventManager() { }
			virtual void DispatchEvents() = 0;
			virtual void DispatchEvent(IEventData& event) = 0;
			virtual void DispatchEvent(EventPtr event) = 0;
			virtual void QueueEvent(IEventData& event) = 0;
			virtual void QueueEvent(EventPtr event) = 0;
			virtual void DequeueFirst(EventType type) = 0;
			virtual void RegisterHandler(EventType type, EventHandlerPtr handler) = 0;
			virtual void DeregisterHandler(EventType type, EventHandlerPtr handler) = 0;
		};
	}
}

#endif