#ifndef ENGINEFWD_H
#define ENGINEFWD_H

#include <memory>

namespace GameEngine
{
	namespace Display
	{
		struct IrrlichtRenderer;
		class MessagingWindow;
		struct MessagingWindowImpl;
	}

	namespace PhysicsEngine
	{
		class IPhysicsEngine;

		class BulletPhysics;
		struct BulletPhysicsData;
		class BulletDebugRenderer;
	}

	namespace LinearAlgebra
	{
		class Mat4;
		class Vec3;

		struct Vec3Impl;
		struct Mat4Impl;
	}

	class GameActor;
	typedef std::shared_ptr<GameActor> StrongActorPtr;
	typedef std::weak_ptr<GameActor> WeakActorPtr;
	typedef unsigned int ActorID;

	class Cube;
	class Sphere;

	class Game;
	struct GameImpl;
}

#endif