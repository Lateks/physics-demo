#pragma once

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
			NONE,
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
			virtual void VDispatchEvent(IEventData& event) = 0;
			virtual void VDispatchEvent(EventPtr event) = 0;
			virtual void VQueueEvent(IEventData& event) = 0;
			virtual void VQueueEvent(EventPtr event) = 0;
			virtual void VDequeueFirst(EventType type) = 0;
			virtual void VRegisterHandler(EventType type, EventHandlerPtr handler) = 0;
			virtual void VDeregisterHandler(EventType type, EventHandlerPtr handler) = 0;
		};
	}
}