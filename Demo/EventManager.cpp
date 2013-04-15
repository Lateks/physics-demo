#include "EventManager.h"
#include "IEventManager.h"
#include <algorithm>

using std::vector;

namespace GameEngine
{
	namespace Events
	{
		EventManager::EventManager() : m_activeQueue(0) { }

		void EventManager::DispatchEvents()
		{
			auto processingQueue = &m_eventQueues[m_activeQueue];
			m_activeQueue = (m_activeQueue + 1) % EventManager::NUM_QUEUES;
			m_eventQueues[m_activeQueue].clear();

			while (!processingQueue->empty())
			{
				EventPtr event = processingQueue->front();
				DispatchEvent(event);
				processingQueue->pop_front();
			}
		}

		void EventManager::DispatchEvent(IEventData& event)
		{
			DispatchEvent(EventPtr(&event));
		}

		void EventManager::DispatchEvent(EventPtr event)
		{
			EventHandlerList *handlerList = &m_eventHandlers[event->GetEventType()];
			std::for_each(handlerList->begin(), handlerList->end(),
				[&event] (EventHandlerPtr handler) { (*handler.get())(event); });
		}

		void EventManager::QueueEvent(IEventData& event)
		{
			QueueEvent(EventPtr(&event));
		}

		void EventManager::QueueEvent(EventPtr event)
		{
			m_eventQueues[m_activeQueue].push_back(event);
		}

		void EventManager::DequeueFirst(EventType type)
		{
			auto activeQueue = m_eventQueues[m_activeQueue];
			auto it = std::find_if(activeQueue.begin(), activeQueue.end(),
				[&type] (EventPtr event) { return event->GetEventType() == type; });
			if (it != activeQueue.end())
			{
				activeQueue.erase(it);
			}
		}

		void EventManager::RegisterHandler(EventType type, EventHandlerPtr handler)
		{
			auto handlers = &m_eventHandlers[type];
			auto it = std::find_if(handlers->begin(), handlers->end(),
				[&handler] (EventHandlerPtr storedHandler) { return handler == storedHandler; });
			if (it == handlers->end()) // only add the handler if it's not already in the list
			{
				handlers->push_back(handler);
			}
		}

		void EventManager::DeregisterHandler(EventType type, EventHandlerPtr handler)
		{
			EventHandlerList handlers = m_eventHandlers[type];
			auto it = std::find_if(handlers.begin(), handlers.end(),
				[&handler] (EventHandlerPtr storedHandler) { return handler == storedHandler; });
			if (it != handlers.end())
			{
				handlers.erase(it);
			}
		}
	}
}