#ifndef ENGINEFWD_H
#define ENGINEFWD_H

#include <memory>

namespace GameEngine
{
	namespace Display
	{
		class IDisplay;
		class IrrlichtDisplay;
		class MessagingWindow;
		class IrrlichtMessagingWindow;
		class IInputState;
	}

	namespace Physics
	{
		class IPhysicsEngine;
		struct MaterialData;
		struct XMLPhysicsData;

		class BulletPhysics;
		class BulletPhysicsConstraint;
		typedef unsigned int ConstraintID;
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
	typedef std::shared_ptr<GameActor> ActorPtr;
	typedef unsigned int ActorID;
	class WorldTransformComponent;

	class Mat4;
	class Vec3;
	class Vec4;
	typedef Vec4 Quaternion; // may be used as a quaternion structure
	typedef Vec4 RGBAColor;

	class Game;
	class GameData;
	class IGameLogic;

	class ITimer;
	class IrrlichtTimer;
}

#endif