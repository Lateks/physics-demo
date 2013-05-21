#pragma once

#include "IEventManager.h"
#include <vector>
#include <deque>
#include <map>
#include <memory>

namespace GameEngine
{
	namespace Events
	{
		struct EventManagerData;

		class EventManager : public IEventManager
		{
		public:
			EventManager();
			virtual ~EventManager();
			virtual void VDispatchEvents() override;
			virtual void VDispatchEvent(EventPtr event) override;
			virtual void VQueueEvent(EventPtr event) override;
			virtual void VDequeueFirst(EventType type) override;
			virtual void VRegisterHandler(EventType type, EventHandlerPtr handler) override;
			virtual void VDeregisterHandler(EventType type, EventHandlerPtr handler) override;
		private:
			EventManager(const EventManager& other);
			EventManager& operator=(const EventManager& other);
			std::unique_ptr<EventManagerData> m_pData;
		};
	}
}