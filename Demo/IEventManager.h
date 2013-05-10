#pragma once

#include <functional>
#include <memory>

namespace GameEngine
{
	namespace Events
	{
		enum class EventType : uint8_t
		{
			NONE = 0, // this can be used as a default value for variables of this enum type
			COLLISION_EVENT,
			SEPARATION_EVENT,
			ACTOR_MOVED,
			ENTER_TRIGGER,
			EXIT_TRIGGER,
			CAMERA_MOVED
		};

		class IEventData
		{
		public:
			virtual ~IEventData() { };
			virtual EventType VGetEventType() const = 0;
			virtual float VGetTimestamp() const = 0;
		};

		typedef std::shared_ptr<IEventData> EventPtr;
		typedef std::function<void(EventPtr)> EventHandler;
		typedef std::shared_ptr<EventHandler> EventHandlerPtr;

		class IEventManager
		{
		public:
			IEventManager() { }
			virtual ~IEventManager() { }
			virtual void VDispatchEvents() = 0;
			virtual void VDispatchEvent(EventPtr event) = 0;
			virtual void VQueueEvent(EventPtr event) = 0;

			// Removes the first event of the given type from the event queue.
			virtual void VDequeueFirst(EventType type) = 0;

			/* Event handlers (callbacks) are always pointers to std::function
			 * objects because std:function objects themselves cannot be compared.
			 * Comparison needs to be possible to make deregistering handlers
			 * work - as well as to make it possible to check that a particular
			 * handler is not registered twice for the same event type.
			 */
			virtual void VRegisterHandler(EventType type, EventHandlerPtr handler) = 0;
			virtual void VDeregisterHandler(EventType type, EventHandlerPtr handler) = 0;
		};
	}
}