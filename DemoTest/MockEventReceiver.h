#pragma once
#include <IEventManager.h>
#include <memory>

namespace DemoTest
{
	struct MockEventReceiverData;

	class MockEventReceiver
	{
	public:
		MockEventReceiver();
		virtual ~MockEventReceiver();
		void RegisterTo(GameEngine::Events::EventType type, std::shared_ptr<GameEngine::Events::IEventManager> pEventManager);
		void DeregisterFrom(GameEngine::Events::EventType type, std::shared_ptr<GameEngine::Events::IEventManager> pEventManager);

		int NumCallsReceived();
		int NumValidCallsReceived();
		int NumInvalidCallsReceived(); // calls that should not have come to this event receiver
	private:
		std::unique_ptr<MockEventReceiverData> pImpl;
	};
}