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
		class EventManager : public IEventManager
		{
		public:
			typedef std::vector<EventHandlerPtr> EventHandlerList;
			typedef std::map<EventType, EventHandlerList> EventTypeToHandlerMap;
			typedef std::deque<EventPtr> EventQueue;

			EventManager();
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
			// Use a two queue system like in McShaffry's book
			// (while one of the queues is being processed, the
			// other one is used to queue new events and dequeue events).
			static const unsigned int NUM_QUEUES = 2;
			unsigned int m_activeQueue;
			EventQueue m_eventQueues[NUM_QUEUES];
			EventTypeToHandlerMap m_eventHandlers;
		};
	}
}

#endif