#ifndef EVENT_MANAGER_H
#define EVENT_MANAGER_H

#include "IEventManager.h"
#include <vector>
#include <deque>
#include <map>
#include <memory>

namespace GameEngine
{
	namespace Events
	{
		class EventManager : IEventManager
		{
		public:
			typedef std::vector<EventHandlerPtr> EventHandlerList;
			typedef std::map<EventType, EventHandlerList> EventTypeToHandlerMap;

			virtual ~EventManager() { }
			virtual void DispatchEvents() override;
			virtual void DispatchEvent(IEventData& event) override;
			virtual void DispatchEvent(EventPtr event) override;
			virtual void QueueEvent(IEventData& event) override;
			virtual void QueueEvent(EventPtr event) override;
			virtual void DequeueFirst(EventType type) override;
			virtual void RegisterHandler(EventType type, EventHandlerPtr handler) override;
			virtual void DeregisterHandler(EventType type, EventHandlerPtr handler) override;
		private:
			std::deque<EventPtr> m_eventQueue;
			EventTypeToHandlerMap m_eventHandlers;
		};
	}
}

#endif