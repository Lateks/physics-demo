#ifndef ENGINEFWD_H
#define ENGINEFWD_H

#include <memory>

namespace GameEngine
{
	namespace Display
	{
		class IDisplay;
		class MessagingWindow;
		class IInputState;
		class IDisplayFactory;
	}

	namespace Physics
	{
		class IPhysicsEngine;
		struct MaterialData;
		struct XMLPhysicsData;
		typedef unsigned int ConstraintID;
		class IPhysicsEngineFactory;
	}

	namespace Events
	{
		class IEventManager;
		class EventManager;
		class IEventData;
	}

	class GameActor;
	typedef std::shared_ptr<GameActor> ActorPtr;
	typedef unsigned int ActorID;
	class WorldTransformComponent;

	class Vec3;
	class Vec4;
	typedef Vec4 Quaternion;
	typedef Vec4 RGBAColor;

	class Game;
	class GameData;
	class IGameLogic;
	class IGameLogicFactory;

	class ITimer;
}

#endif