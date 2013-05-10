#include "CppUnitTest.h"
#include <EventManager.h>
#include <Events.h>
#include <vector>
#include <algorithm>
#include <functional>

using namespace Microsoft::VisualStudio::CppUnitTestFramework;
using namespace GameEngine::Events;

namespace DemoTest
{		
	TEST_CLASS(EventSystemTest)
	{
	public:
		typedef std::shared_ptr<IEventManager> EventManagerPtr;
		typedef std::vector<std::shared_ptr<IEventManager>> EventManagerList;

		struct MockEventHandler
		{
			MockEventHandler(EventType eventType)
				: num_calls(0), num_valid_calls(0), num_invalid_calls(0), expectedType(eventType),
				eventHandler(), currentRegistrations()
			{
				eventHandler.reset(new std::function<void(EventPtr)>([this] (EventPtr event) { HandleEvent(event); }));
			}

			void RegisterTo(EventManagerPtr pEventManager)
			{
				pEventManager->VRegisterHandler(expectedType, eventHandler);
				auto registeredManager = FindEventManager(pEventManager);
				if (registeredManager == currentRegistrations.end())
				{
					currentRegistrations.push_back(pEventManager);
				}
			}

			void DeregisterFrom(EventManagerPtr pEventManager)
			{
				pEventManager->VDeregisterHandler(expectedType, eventHandler);
				auto registeredManager = FindEventManager(pEventManager);
				if (registeredManager != currentRegistrations.end())
				{
					currentRegistrations.erase(registeredManager);
				}
			}

			void HandleEvent(EventPtr event)
			{
				if (event->VGetEventType() == expectedType && !currentRegistrations.empty())
				{
					++num_valid_calls;
				}
				else
				{
					++num_invalid_calls;
				}
				++num_calls;
			}

			EventManagerList::iterator FindEventManager(std::shared_ptr<IEventManager> pEventManager)
			{
				return std::find(currentRegistrations.begin(), currentRegistrations.end(), pEventManager);
			}

			int num_calls;
			int num_valid_calls;
			int num_invalid_calls; // calls that should not have come to this event handler
			const EventType expectedType;
			EventHandlerPtr eventHandler;
			EventManagerList currentRegistrations;
		};

		TEST_METHOD(SingleHandlerReceivesEventsRegisteredTo)
		{
			auto eventMgr = std::make_shared<EventManager>();
			MockEventHandler moveHandler(EventType::ACTOR_MOVED);
			moveHandler.RegisterTo(eventMgr);

			eventMgr->VQueueEvent(std::make_shared<ActorMoveEvent>(0, 1));
			eventMgr->VDispatchEvents();
			Assert::AreEqual(moveHandler.num_valid_calls, 1);
		}

		TEST_METHOD(SingleHandlerReceivesEventsWithImmediateDispatch)
		{
			auto eventMgr = std::make_shared<EventManager>();
			MockEventHandler moveHandler(EventType::ACTOR_MOVED);
			moveHandler.RegisterTo(eventMgr);

			eventMgr->VDispatchEvent(std::make_shared<ActorMoveEvent>(0, 1));
			Assert::AreEqual(moveHandler.num_valid_calls, 1);
		}

		TEST_METHOD(EventsAreClearedAfterDispatch)
		{
			auto eventMgr = std::make_shared<EventManager>();
			MockEventHandler moveHandler(EventType::ACTOR_MOVED);
			moveHandler.RegisterTo(eventMgr);

			eventMgr->VQueueEvent(std::make_shared<ActorMoveEvent>(0, 1));
			eventMgr->VDispatchEvents();
			eventMgr->VDispatchEvents();
			eventMgr->VDispatchEvents();
			Assert::AreEqual(moveHandler.num_valid_calls, 1);
		}

		TEST_METHOD(HandlerDoesNotReceiveEventsNotRegisteredTo)
		{
			auto eventMgr = std::make_shared<EventManager>();
			MockEventHandler moveHandler(EventType::CAMERA_MOVED);
			moveHandler.RegisterTo(eventMgr);

			eventMgr->VQueueEvent(std::make_shared<ActorMoveEvent>(0, 1));
			eventMgr->VDispatchEvents();
			Assert::AreEqual(moveHandler.num_calls, 0);
		}

		TEST_METHOD(MultipleHandlersCanReceiveTheSameEvent)
		{
			auto eventMgr = std::make_shared<EventManager>();
			MockEventHandler triggerHandler1(EventType::ENTER_TRIGGER);
			triggerHandler1.RegisterTo(eventMgr);
			MockEventHandler triggerHandler2(EventType::ENTER_TRIGGER);
			triggerHandler2.RegisterTo(eventMgr);

			eventMgr->VQueueEvent(std::make_shared<TriggerEntryEvent>(0, 1, 2));
			eventMgr->VDispatchEvents();
			Assert::AreEqual(triggerHandler1.num_valid_calls, 1);
			Assert::AreEqual(triggerHandler2.num_valid_calls, 1);
		}

		TEST_METHOD(HandlerDoesNotReceiveEventsAfterDeregistering)
		{
			auto eventMgr = std::make_shared<EventManager>();
			MockEventHandler triggerHandler1(EventType::ENTER_TRIGGER);
			triggerHandler1.RegisterTo(eventMgr);
			MockEventHandler triggerHandler2(EventType::ENTER_TRIGGER);
			triggerHandler2.RegisterTo(eventMgr);

			triggerHandler1.DeregisterFrom(eventMgr);
			eventMgr->VQueueEvent(std::make_shared<TriggerEntryEvent>(0, 1, 2));
			eventMgr->VDispatchEvents();
			Assert::AreEqual(triggerHandler1.num_calls, 0);
			Assert::AreEqual(triggerHandler2.num_valid_calls, 1);
		}

		TEST_METHOD(HandlerIsOnlyEverRegisteredOnce)
		{
			auto eventMgr = std::make_shared<EventManager>();
			MockEventHandler moveHandler(EventType::ACTOR_MOVED);
			moveHandler.RegisterTo(eventMgr);
			moveHandler.RegisterTo(eventMgr);
			moveHandler.RegisterTo(eventMgr);

			eventMgr->VDispatchEvent(std::make_shared<ActorMoveEvent>(0, 1));
			Assert::AreEqual(moveHandler.num_valid_calls, 1);
		}
	};
}