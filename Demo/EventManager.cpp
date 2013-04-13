#include "EventManager.h"
#include "IEventManager.h"
#include <algorithm>

using std::vector;

namespace GameEngine
{
	namespace Events
	{
		void EventManager::DispatchEvents()
		{
			while (!m_eventQueue.empty())
			{
				EventPtr event = m_eventQueue.front();
				DispatchEvent(event);
				m_eventQueue.pop_front();
			}
		}

		void EventManager::DispatchEvent(IEventData& event)
		{
			DispatchEvent(EventPtr(&event));
		}

		void EventManager::DispatchEvent(EventPtr event)
		{
			EventHandlerList handlerList = m_eventHandlers[event->GetEventType()];
			std::for_each(handlerList.begin(), handlerList.end(),
				[&event] (EventHandlerPtr handler) { (*handler.get())(event); });
		}

		void EventManager::QueueEvent(IEventData& event)
		{
			QueueEvent(EventPtr(&event));
		}

		void EventManager::QueueEvent(EventPtr event)
		{
			m_eventQueue.push_back(event);
		}

		void EventManager::DequeueFirst(EventType type)
		{
			auto it = std::find_if(m_eventQueue.begin(), m_eventQueue.end(),
				[&type] (EventPtr event) { return event->GetEventType() == type; });
			if (it != m_eventQueue.end())
			{
				m_eventQueue.erase(it);
			}
		}

		void EventManager::RegisterHandler(EventType type, EventHandlerPtr handler)
		{
			auto handlers = m_eventHandlers[type];
			auto it = std::find_if(handlers.begin(), handlers.end(),
				[&handler] (EventHandlerPtr storedHandler) { return handler == storedHandler; });
			if (it != handlers.end())
			{
				handlers.push_back(handler);
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