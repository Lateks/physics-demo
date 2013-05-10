#include "CppUnitTest.h"
#include <EventManager.h>
#include <Events.h>
#include <vector>
#include <map>
#include <algorithm>
#include <functional>

using namespace Microsoft::VisualStudio::CppUnitTestFramework;
using namespace GameEngine::Events;

namespace DemoTest
{
	typedef std::shared_ptr<IEventManager> EventManagerPtr;
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

	TEST_CLASS(EventSystemTest)
	{
	public:
		struct MockEventReceiver
		{
			MockEventReceiver()
				: num_calls(0), num_valid_calls(0), num_invalid_calls(0),
				eventHandler(), currentRegistrations(), expectedEventTypes()
			{
				eventHandler.reset(new std::function<void(EventPtr)>([this] (EventPtr event) { HandleEvent(event); }));
			}

			void RegisterTo(EventType type, EventManagerPtr pEventManager)
			{
				pEventManager->VRegisterHandler(type, eventHandler);
				AddToVectorMap<EventManagerPtr, EventType>(currentRegistrations, pEventManager, type);
				AddToVectorMap<EventType, EventManagerPtr>(expectedEventTypes, type, pEventManager);
			}

			void DeregisterFrom(EventType type, EventManagerPtr pEventManager)
			{
				pEventManager->VDeregisterHandler(type, eventHandler);
				RemoveFromVectorMap<EventManagerPtr, EventType>(currentRegistrations, pEventManager, type);
				RemoveFromVectorMap<EventType, EventManagerPtr>(expectedEventTypes, type, pEventManager);
			}

			void HandleEvent(EventPtr event)
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

			bool IsExpectedType(EventType type)
			{
				return expectedEventTypes.find(type) != expectedEventTypes.end();
			}

			int num_calls;
			int num_valid_calls;
			int num_invalid_calls; // calls that should not have come to this event handler
			EventHandlerPtr eventHandler;
			EventManagerToTypeMap currentRegistrations;
			EventTypeToManagerMap expectedEventTypes;
		};

		TEST_METHOD(SingleHandlerReceivesEventsRegisteredTo)
		{
			auto eventMgr = std::make_shared<EventManager>();
			MockEventReceiver moveHandler;
			moveHandler.RegisterTo(EventType::ACTOR_MOVED, eventMgr);

			eventMgr->VQueueEvent(std::make_shared<ActorMoveEvent>(0, 1));
			eventMgr->VDispatchEvents();
			Assert::AreEqual(1, moveHandler.num_valid_calls);
		}

		TEST_METHOD(SingleHandlerReceivesEventsWithImmediateDispatch)
		{
			auto eventMgr = std::make_shared<EventManager>();
			MockEventReceiver moveHandler;
			moveHandler.RegisterTo(EventType::ACTOR_MOVED, eventMgr);

			eventMgr->VDispatchEvent(std::make_shared<ActorMoveEvent>(0, 1));
			Assert::AreEqual(1, moveHandler.num_valid_calls);
		}

		TEST_METHOD(EventsAreClearedAfterDispatch)
		{
			auto eventMgr = std::make_shared<EventManager>();
			MockEventReceiver moveHandler;
			moveHandler.RegisterTo(EventType::ACTOR_MOVED, eventMgr);

			eventMgr->VQueueEvent(std::make_shared<ActorMoveEvent>(0, 1));
			eventMgr->VDispatchEvents();
			eventMgr->VDispatchEvents();
			eventMgr->VDispatchEvents();
			Assert::AreEqual(1, moveHandler.num_valid_calls);
		}

		TEST_METHOD(HandlerDoesNotReceiveEventsNotRegisteredTo)
		{
			auto eventMgr = std::make_shared<EventManager>();
			MockEventReceiver moveHandler;
			moveHandler.RegisterTo(EventType::CAMERA_MOVED, eventMgr);

			eventMgr->VQueueEvent(std::make_shared<ActorMoveEvent>(0, 1));
			eventMgr->VDispatchEvents();
			Assert::AreEqual(0, moveHandler.num_calls);
		}

		TEST_METHOD(MultipleHandlersCanReceiveTheSameEvent)
		{
			auto eventType = EventType::ENTER_TRIGGER;
			auto eventMgr = std::make_shared<EventManager>();
			MockEventReceiver triggerHandler1;
			triggerHandler1.RegisterTo(eventType, eventMgr);
			MockEventReceiver triggerHandler2;
			triggerHandler2.RegisterTo(eventType, eventMgr);

			eventMgr->VQueueEvent(std::make_shared<TriggerEntryEvent>(0, 1, 2));
			eventMgr->VDispatchEvents();
			Assert::AreEqual(1, triggerHandler1.num_valid_calls);
			Assert::AreEqual(1, triggerHandler2.num_valid_calls);
		}

		TEST_METHOD(HandlerDoesNotReceiveEventsAfterDeregistering)
		{
			auto eventType = EventType::ENTER_TRIGGER;
			auto eventMgr = std::make_shared<EventManager>();
			MockEventReceiver triggerHandler1;
			triggerHandler1.RegisterTo(eventType, eventMgr);
			MockEventReceiver triggerHandler2;
			triggerHandler2.RegisterTo(eventType, eventMgr);

			triggerHandler1.DeregisterFrom(eventType, eventMgr);
			eventMgr->VQueueEvent(std::make_shared<TriggerEntryEvent>(0, 1, 2));
			eventMgr->VDispatchEvents();
			Assert::AreEqual(0, triggerHandler1.num_calls);
			Assert::AreEqual(1, triggerHandler2.num_valid_calls);
		}

		TEST_METHOD(HandlerIsOnlyEverRegisteredOnce)
		{
			auto eventType = EventType::ACTOR_MOVED;
			auto eventMgr = std::make_shared<EventManager>();
			MockEventReceiver moveHandler;
			moveHandler.RegisterTo(eventType, eventMgr);
			moveHandler.RegisterTo(eventType, eventMgr);
			moveHandler.RegisterTo(eventType, eventMgr);

			eventMgr->VDispatchEvent(std::make_shared<ActorMoveEvent>(0, 1));
			Assert::AreEqual(1, moveHandler.num_valid_calls);
		}

		TEST_METHOD(SameHandlerMayRegisterForSeveralDifferentEvents)
		{
			auto eventMgr = std::make_shared<EventManager>();
			MockEventReceiver eventHandler;
			eventHandler.RegisterTo(EventType::ACTOR_MOVED, eventMgr);
			eventHandler.RegisterTo(EventType::ENTER_TRIGGER, eventMgr);
			eventHandler.RegisterTo(EventType::EXIT_TRIGGER, eventMgr);

			eventMgr->VQueueEvent(std::make_shared<ActorMoveEvent>(0, 1));
			eventMgr->VQueueEvent(std::make_shared<TriggerEntryEvent>(0, 1, 2));
			eventMgr->VQueueEvent(std::make_shared<TriggerExitEvent>(0, 1, 2));
			eventMgr->VDispatchEvents();
			Assert::AreEqual(3, eventHandler.num_valid_calls);
		}

		TEST_METHOD(NewEventsQueuedDuringEventDispatchAreSentAtNextDispatch)
		{
			auto eventMgr = std::make_shared<EventManager>();
			bool eventAdderCalled = false;
			auto eventAdder = std::make_shared<std::function<void(EventPtr)>>(
				[&eventMgr, &eventAdderCalled] (EventPtr event) {
					eventAdderCalled = true;
					eventMgr->VQueueEvent(std::make_shared<ActorMoveEvent>(0, 1));
			});

			eventMgr->VRegisterHandler(EventType::ENTER_TRIGGER, eventAdder);
			eventMgr->VQueueEvent(std::make_shared<TriggerEntryEvent>(0, 1, 2));
			eventMgr->VDispatchEvents();
			Assert::IsTrue(eventAdderCalled);
			eventMgr->VDeregisterHandler(EventType::ENTER_TRIGGER, eventAdder);

			MockEventReceiver eventHandler;
			eventHandler.RegisterTo(EventType::ACTOR_MOVED, eventMgr);
			eventMgr->VDispatchEvents();
			Assert::AreEqual(1, eventHandler.num_valid_calls);

		}

		TEST_METHOD(CanDequeueNewEvents)
		{
			auto eventMgr = std::make_shared<EventManager>();
			bool eventAdderCalled = false;
			auto eventAdder = std::make_shared<std::function<void(EventPtr)>>(
				[&eventMgr, &eventAdderCalled] (EventPtr event) {
					eventAdderCalled = true; eventMgr->VQueueEvent(std::make_shared<ActorMoveEvent>(0, 1));
			});
			bool eventRemoverCalled = false;
			auto eventRemover = std::make_shared<std::function<void(EventPtr)>>(
				[&eventMgr, &eventRemoverCalled] (EventPtr event) {
					eventRemoverCalled = true; eventMgr->VDequeueFirst(EventType::ACTOR_MOVED);
			});

			eventMgr->VRegisterHandler(EventType::ACTOR_MOVED, eventAdder);
			eventMgr->VRegisterHandler(EventType::ACTOR_MOVED, eventRemover);
			eventMgr->VQueueEvent(std::make_shared<ActorMoveEvent>(0, 1));
			eventMgr->VDispatchEvents();
			Assert::IsTrue(eventAdderCalled && eventRemoverCalled);

			MockEventReceiver eventHandler;
			eventHandler.RegisterTo(EventType::ACTOR_MOVED, eventMgr);
			eventMgr->VDispatchEvents();
			Assert::AreEqual(0, eventHandler.num_calls);
		}

		TEST_METHOD(DequeuingEventsDuringDispatchDoesNotAffectEventsBeingDispatched)
		{
			auto eventMgr = std::make_shared<EventManager>();
			bool customHandlerCalled = false;
			auto customHandler = std::make_shared<std::function<void(EventPtr)>>(
				[&eventMgr, &customHandlerCalled] (EventPtr event) {
					customHandlerCalled = true; eventMgr->VDequeueFirst(EventType::ACTOR_MOVED);
			});
			eventMgr->VRegisterHandler(EventType::ENTER_TRIGGER, customHandler);

			MockEventReceiver eventHandler;
			eventHandler.RegisterTo(EventType::ACTOR_MOVED, eventMgr);

			eventMgr->VQueueEvent(std::make_shared<TriggerEntryEvent>(0, 1, 2));
			eventMgr->VQueueEvent(std::make_shared<ActorMoveEvent>(0, 1));
			eventMgr->VDispatchEvents();
			Assert::IsTrue(customHandlerCalled);
			Assert::AreEqual(1, eventHandler.num_valid_calls);
		}
	};
}