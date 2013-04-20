#ifndef ENGINEFWD_H
#define ENGINEFWD_H

#include <memory>

namespace GameEngine
{
	namespace Display
	{
		class IDisplay;
		class IrrlichtDisplay;
		struct IrrlichtDisplayImpl;
		class IrrlichtInputState;
		class MessagingWindow;
		struct MessagingWindowImpl;
		class IInputState;
	}

	namespace Physics
	{
		class IPhysicsEngine;
		struct MaterialData;
		struct XMLPhysicsData;

		class BulletPhysics;
		struct BulletPhysicsData;
		class BulletDebugRenderer;
	}

	namespace Events
	{
		class IEventManager;
		class EventManager;
		class IEventData;
		class BaseEventData;
		class CollisionEvent;
		class SeparationEvent;
		class ActorMoveEvent;
	}

	class GameActor;
	typedef std::shared_ptr<GameActor> StrongActorPtr;
	typedef std::weak_ptr<GameActor> WeakActorPtr;
	typedef unsigned int ActorID;
	class WorldTransformComponent;

	class Mat4;
	class Vec3;
	class Vec4;

	class Game;
	class GameData;

	class ITimer;
	class IrrlichtTimer;
}

#endif