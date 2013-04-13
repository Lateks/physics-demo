#ifndef ENGINEFWD_H
#define ENGINEFWD_H

#include <memory>

namespace GameEngine
{
	namespace Display
	{
		class IRenderer;
		class IrrlichtRenderer;
		struct IrrlichtRendererImpl;
		class MessagingWindow;
		struct MessagingWindowImpl;
	}

	namespace PhysicsEngine
	{
		class IPhysicsEngine;
		struct MaterialData;
		struct XMLPhysicsData;

		class BulletPhysics;
		struct BulletPhysicsData;
		class BulletDebugRenderer;
	}

	namespace LinearAlgebra
	{
		class Mat4;
		class Vec3;

		struct Mat4Impl;
	}

	class GameActor;
	typedef std::shared_ptr<GameActor> StrongActorPtr;
	typedef std::weak_ptr<GameActor> WeakActorPtr;
	typedef unsigned int ActorID;
	class WorldTransformComponent;

	class Game;
	class GameData;

	class ITimer;
	class IrrlichtTimer;
}

#endif