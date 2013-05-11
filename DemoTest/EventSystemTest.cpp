#include "CppUnitTest.h"
#include "MockEventReceiver.h"
#include <EventManager.h>
#include <Events.h>
#include <vector>
#include <functional>

using namespace Microsoft::VisualStudio::CppUnitTestFramework;
using namespace GameEngine::Events;

namespace DemoTest
{
	typedef std::shared_ptr<IEventManager> EventManagerPtr;

	TEST_CLASS(EventSystemTest)
	{
	public:

		TEST_METHOD(SingleHandlerReceivesEventsRegisteredTo)
		{
			auto eventMgr = std::make_shared<EventManager>();
			MockEventReceiver moveHandler;
			moveHandler.RegisterTo(EventType::ACTOR_MOVED, eventMgr);

			eventMgr->VQueueEvent(std::make_shared<ActorMoveEvent>(0, 1));
			eventMgr->VDispatchEvents();
			Assert::AreEqual(1, moveHandler.NumValidCallsReceived());
		}

		TEST_METHOD(SingleHandlerReceivesEventsWithImmediateDispatch)
		{
			auto eventMgr = std::make_shared<EventManager>();
			MockEventReceiver moveHandler;
			moveHandler.RegisterTo(EventType::ACTOR_MOVED, eventMgr);

			eventMgr->VDispatchEvent(std::make_shared<ActorMoveEvent>(0, 1));
			Assert::AreEqual(1, moveHandler.NumValidCallsReceived());
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
			Assert::AreEqual(1, moveHandler.NumValidCallsReceived());
		}

		TEST_METHOD(HandlerDoesNotReceiveEventsNotRegisteredTo)
		{
			auto eventMgr = std::make_shared<EventManager>();
			MockEventReceiver moveHandler;
			moveHandler.RegisterTo(EventType::CAMERA_MOVED, eventMgr);

			eventMgr->VQueueEvent(std::make_shared<ActorMoveEvent>(0, 1));
			eventMgr->VDispatchEvents();
			Assert::AreEqual(0, moveHandler.NumCallsReceived());
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
			Assert::AreEqual(1, triggerHandler1.NumValidCallsReceived());
			Assert::AreEqual(1, triggerHandler2.NumValidCallsReceived());
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
			Assert::AreEqual(0, triggerHandler1.NumCallsReceived());
			Assert::AreEqual(1, triggerHandler2.NumValidCallsReceived());
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
			Assert::AreEqual(1, moveHandler.NumValidCallsReceived());
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
			Assert::AreEqual(3, eventHandler.NumValidCallsReceived());
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
			Assert::AreEqual(1, eventHandler.NumValidCallsReceived());

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
			Assert::AreEqual(0, eventHandler.NumValidCallsReceived());
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
			Assert::AreEqual(1, eventHandler.NumValidCallsReceived());
		}
	};
}