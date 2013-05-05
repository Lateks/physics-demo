#include "EventManager.h"
#include "IEventManager.h"
#include <algorithm>
#include <iostream>

using std::vector;

namespace GameEngine
{
	namespace Events
	{
		EventManager::EventManager() : m_activeQueue(0) { }

		void EventManager::VDispatchEvents()
		{
			auto processingQueue = &m_eventQueues[m_activeQueue];
			m_activeQueue = (m_activeQueue + 1) % EventManager::NUM_QUEUES;
			m_eventQueues[m_activeQueue].clear();

			while (!processingQueue->empty())
			{
				EventPtr event = processingQueue->front();
				VDispatchEvent(event);
				processingQueue->pop_front();
			}
		}

		void EventManager::VDispatchEvent(IEventData& event)
		{
			VDispatchEvent(EventPtr(&event));
		}

		void EventManager::VDispatchEvent(EventPtr event)
		{
			EventHandlerList *handlerList = &m_eventHandlers[event->VGetEventType()];
			std::for_each(handlerList->begin(), handlerList->end(),
				[&event] (EventHandlerPtr handler) { (*handler)(event); });
		}

		void EventManager::VQueueEvent(IEventData& event)
		{
			VQueueEvent(EventPtr(&event));
		}

		void EventManager::VQueueEvent(EventPtr event)
		{
			m_eventQueues[m_activeQueue].push_back(event);
		}

		void EventManager::VDequeueFirst(EventType type)
		{
			auto activeQueue = m_eventQueues[m_activeQueue];
			auto it = std::find_if(activeQueue.begin(), activeQueue.end(),
				[&type] (EventPtr event) { return event->VGetEventType() == type; });
			if (it != activeQueue.end())
			{
				activeQueue.erase(it);
			}
		}

		void EventManager::VRegisterHandler(EventType type, EventHandlerPtr handler)
		{
			auto handlers = &m_eventHandlers[type];
			auto it = std::find_if(handlers->begin(), handlers->end(),
				[&handler] (EventHandlerPtr storedHandler) { return handler == storedHandler; });
			if (it == handlers->end()) // only add the handler if it's not already in the list
			{
				handlers->push_back(handler);
			}
		}

		void EventManager::VDeregisterHandler(EventType type, EventHandlerPtr handler)
		{
			EventHandlerList& handlers = m_eventHandlers[type];
			auto it = std::find_if(handlers.begin(), handlers.end(),
				[&handler] (EventHandlerPtr storedHandler) { return handler == storedHandler; });
			if (it != handlers.end())
			{
				handlers.erase(it);
			}
		}
	}
}