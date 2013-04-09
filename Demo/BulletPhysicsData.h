#ifndef BULLET_PHYSICS_DATA_H
#define BULLET_PHYSICS_DATA_H

#include "enginefwd.h"
#include <btBulletCollisionCommon.h>
#include <btBulletDynamicsCommon.h>
#include <map>
#include <set>

/* This is mostly from the McShaffry chapter on integrating
 * the Bullet SDK (the private parts of the BulletPhysics class
 * extracted into a PIMPL struct) with slight variations.
 */
namespace GameEngine
{
	namespace PhysicsEngine
	{
		typedef std::pair<btRigidBody const *, btRigidBody const *> CollisionPair;
		typedef std::set<CollisionPair> CollisionPairs;

		struct BulletPhysicsData
		{
			virtual ~BulletPhysicsData();

			btDynamicsWorld *m_pDynamicsWorld;             // - manages the other required components
			btBroadphaseInterface *m_pCollisionBroadPhase; // - manages the first (rough) phase of collision detection
			btCollisionDispatcher *m_pCollisionDispatcher; // - manages the more accurate second phase of collision detection
			btConstraintSolver *m_pConstraintSolver;       // - manages objects' freedom of motion
			btDefaultCollisionConfiguration *m_pCollisionConfig; // - memory usage configuration
			
			BulletDebugRenderer *m_pDebugRenderer;

			// TODO: reading material and density information from an xml file
			// This should actually be in a completely separate struct since
			// it has nothing to do with Bullet specifically.

			// Store the rigid bodies related to game actors.
			std::map<ActorID, btRigidBody const*> m_actorToRigidBodyMap;
			std::map<btRigidBody const*, ActorID> m_RigidBodyToActorMap;
			btRigidBody *GetRigidBody(ActorID id) const;
			ActorID GetActorID(btRigidBody const *pBody) const;

			CollisionPairs m_PreviousTickCollisions;

			void SendNewCollisionEvent(btPersistentManifold const * const manifold,
				btRigidBody const * const pBody1, btRigidBody const * const pBody2);
			void SendSeparationEvent(btRigidBody const * pBody1, btRigidBody const * pBody2);

			void AddShape(StrongActorPtr pActor, btCollisionShape *shape,
				float mass, const std::string& physicsMaterial);

			static void BulletInternalTickCallback(btDynamicsWorld * const pWorld, btScalar const timeStep);
		};
	}
}

#endif