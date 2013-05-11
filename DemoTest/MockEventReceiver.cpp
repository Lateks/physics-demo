#include "MockEventReceiver.h"
#include <IEventManager.h>
#include <vector>
#include <map>
#include <algorithm>

using namespace GameEngine::Events;

namespace DemoTest
{
	typedef std::shared_ptr<GameEngine::Events::IEventManager> EventManagerPtr;
	typedef std::map<EventManagerPtr, std::vector<EventType>> EventManagerToTypeMap;
	typedef std::map<EventType, std::vector<EventManagerPtr>> EventTypeToManagerMap;

	template < typename E, typename T >
	void RemoveFromVectorMap(std::map<E, std::vector<T>>& map, E key, T value)
	{
		auto iter = map.find(key);
		if (iter != map.end())
		{
			std::remove_if(iter->second.begin(), iter->second.end(),
				[&value] (T other) { return value == other; });
			if (iter->second.empty())
			{
				map.erase(key);
			}
		}
	}

	template < typename E, typename T >
	void AddToVectorMap(std::map<E, std::vector<T>>& map, E key, T value)
	{
		auto &vec = map[key];
		if (std::find(vec.begin(), vec.end(), value) == vec.end())
		{
			map[key].push_back(value);
		}
	}

	struct MockEventReceiverData
	{
		MockEventReceiverData()
			: num_calls(0), num_valid_calls(0), num_invalid_calls(0),
			eventHandler(), currentRegistrations(), expectedEventTypes() { }
		void HandleEvent(GameEngine::Events::EventPtr event);
		bool IsExpectedType(GameEngine::Events::EventType type);

		int num_calls;
		int num_valid_calls;
		int num_invalid_calls;
		GameEngine::Events::EventHandlerPtr eventHandler;
		EventManagerToTypeMap currentRegistrations;
		EventTypeToManagerMap expectedEventTypes;
	};

	MockEventReceiver::MockEventReceiver()
		: pImpl(new MockEventReceiverData())
	{
		pImpl->eventHandler.reset(new std::function<void(EventPtr)>(
			[this] (EventPtr event) { pImpl->HandleEvent(event); }));
	}

	MockEventReceiver::~MockEventReceiver() { }

	void MockEventReceiver::RegisterTo(GameEngine::Events::EventType type, EventManagerPtr pEventManager)
	{
		pEventManager->VRegisterHandler(type, pImpl->eventHandler);
		AddToVectorMap<EventManagerPtr, EventType>(pImpl->currentRegistrations, pEventManager, type);
		AddToVectorMap<EventType, EventManagerPtr>(pImpl->expectedEventTypes, type, pEventManager);
	}

	void MockEventReceiver::DeregisterFrom(GameEngine::Events::EventType type, EventManagerPtr pEventManager)
	{
		pEventManager->VDeregisterHandler(type, pImpl->eventHandler);
		RemoveFromVectorMap<EventManagerPtr, EventType>(pImpl->currentRegistrations, pEventManager, type);
		RemoveFromVectorMap<EventType, EventManagerPtr>(pImpl->expectedEventTypes, type, pEventManager);
	}

	int MockEventReceiver::NumCallsReceived()
	{
		return pImpl->num_calls;
	}

	int MockEventReceiver::NumValidCallsReceived()
	{
		return pImpl->num_valid_calls;
	}

	int MockEventReceiver::NumInvalidCallsReceived()
	{
		return pImpl->num_invalid_calls;
	}

	void MockEventReceiverData::HandleEvent(GameEngine::Events::EventPtr event)
	{
		if (IsExpectedType(event->VGetEventType()) && !currentRegistrations.empty())
		{
			++num_valid_calls;
		}
		else
		{
			++num_invalid_calls;
		}
		++num_calls;
	}

	bool MockEventReceiverData::IsExpectedType(GameEngine::Events::EventType type)
	{
		return expectedEventTypes.find(type) != expectedEventTypes.end();
	}
}