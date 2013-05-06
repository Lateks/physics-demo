#include "EventManager.h"
#include "IEventManager.h"
#include <algorithm>
#include <iostream>

using std::vector;

namespace GameEngine
{
	namespace Events
	{
		typedef std::vector<EventHandlerPtr> EventHandlerList;
		typedef std::map<EventType, EventHandlerList> EventTypeToHandlerMap;
		typedef std::deque<EventPtr> EventQueue;

		struct EventManagerData
		{
			// Use a two queue system like in McShaffry's book
			// (while one of the queues is being processed, the
			// other one is used to queue new events and dequeue events).
			static const unsigned int NUM_QUEUES = 2;
			unsigned int m_activeQueue;
			EventQueue m_eventQueues[NUM_QUEUES];
			EventTypeToHandlerMap m_eventHandlers;
		};

		EventManager::EventManager() : m_pData(new EventManagerData())
		{
			m_pData->m_activeQueue = 0;
		}

		EventManager::~EventManager() { }

		void EventManager::VDispatchEvents()
		{
			auto processingQueue = &m_pData->m_eventQueues[m_pData->m_activeQueue];
			m_pData->m_activeQueue = (m_pData->m_activeQueue + 1) % EventManagerData::NUM_QUEUES;
			m_pData->m_eventQueues[m_pData->m_activeQueue].clear();

			while (!processingQueue->empty())
			{
				EventPtr event = processingQueue->front();
				VDispatchEvent(event);
				processingQueue->pop_front();
			}
		}

		void EventManager::VDispatchEvent(EventPtr event)
		{
			EventHandlerList *handlerList = &m_pData->m_eventHandlers[event->VGetEventType()];
			std::for_each(handlerList->begin(), handlerList->end(),
				[&event] (EventHandlerPtr handler) { (*handler)(event); });
		}

		void EventManager::VQueueEvent(EventPtr event)
		{
			m_pData->m_eventQueues[m_pData->m_activeQueue].push_back(event);
		}

		void EventManager::VDequeueFirst(EventType type)
		{
			auto activeQueue = m_pData->m_eventQueues[m_pData->m_activeQueue];
			auto it = std::find_if(activeQueue.begin(), activeQueue.end(),
				[&type] (EventPtr event) { return event->VGetEventType() == type; });
			if (it != activeQueue.end())
			{
				activeQueue.erase(it);
			}
		}

		void EventManager::VRegisterHandler(EventType type, EventHandlerPtr handler)
		{
			auto handlers = &m_pData->m_eventHandlers[type];
			auto it = std::find_if(handlers->begin(), handlers->end(),
				[&handler] (EventHandlerPtr storedHandler) { return handler == storedHandler; });
			if (it == handlers->end()) // only add the handler if it's not already in the list
			{
				handlers->push_back(handler);
			}
		}

		void EventManager::VDeregisterHandler(EventType type, EventHandlerPtr handler)
		{
			EventHandlerList& handlers = m_pData->m_eventHandlers[type];
			auto it = std::find_if(handlers.begin(), handlers.end(),
				[&handler] (EventHandlerPtr storedHandler) { return handler == storedHandler; });
			if (it != handlers.end())
			{
				handlers.erase(it);
			}
		}
	}
}