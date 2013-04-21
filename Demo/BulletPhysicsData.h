#ifndef BULLET_PHYSICS_DATA_H
#define BULLET_PHYSICS_DATA_H

#include "enginefwd.h"
#include "BulletPhysicsObject.h"
#include <btBulletCollisionCommon.h>
#include <btBulletDynamicsCommon.h>
#include <map>
#include <set>
#include <vector>

/* This is mostly from the McShaffry chapter on integrating
 * the Bullet SDK (the private parts of the BulletPhysics class
 * extracted into a PIMPL struct) with slight variations.
 */
namespace GameEngine
{
	namespace Physics
	{
		typedef std::pair<btRigidBody const *, btRigidBody const *> CollisionPair;
		typedef std::set<CollisionPair> CollisionPairs;

		struct BulletPhysicsData
		{
		public:
			BulletPhysicsData()
				: m_pDynamicsWorld(nullptr), m_pCollisionBroadPhase(nullptr),
				m_pCollisionDispatcher(nullptr), m_pCollisionConfig(nullptr),
				m_pConstraintSolver(nullptr), m_pDebugRenderer(nullptr) { }
			virtual bool VInitializeSystems();
			virtual ~BulletPhysicsData();

			btDynamicsWorld *m_pDynamicsWorld;             // - manages the other required components
			btBroadphaseInterface *m_pCollisionBroadPhase; // - manages the first (rough) phase of collision detection
			btCollisionDispatcher *m_pCollisionDispatcher; // - manages the more accurate second phase of collision detection
			btConstraintSolver *m_pConstraintSolver;       // - manages objects' freedom of motion
			btDefaultCollisionConfiguration *m_pCollisionConfig; // - memory usage configuration
			
			BulletDebugRenderer *m_pDebugRenderer;

			XMLPhysicsData *m_physicsMaterialData;

			/* Store the rigid bodies related to game actors.
			 * Several rigid bodies can be related to a single actor, but only
			 * a single actor may be related to any rigid body. At the moment
			 * only "static" actors (basically map elements) can own several
			 * rigid bodies.
			 */
			std::map<ActorID, std::shared_ptr<BulletPhysicsObject>> m_actorToBulletPhysicsObjectMap;
			std::map<const btRigidBody*, ActorID> m_rigidBodyToActorMap;

			std::shared_ptr<BulletPhysicsObject> GetPhysicsObject(ActorID id) const;
			ActorID GetActorID(const btRigidBody *pBody) const;

			CollisionPairs m_previousTickCollisions;

			void SendNewCollisionEvent(const btPersistentManifold * manifold,
				const btRigidBody * pBody1, const btRigidBody * pBody2);
			void SendSeparationEvent(const btRigidBody * pBody1, const btRigidBody * pBody2);

			void AddShape(StrongActorPtr pActor, btCollisionShape *shape,
				float mass, const std::string& physicsMaterial);
			void AddStaticColliderShape(StrongActorPtr pActor, btCollisionShape *shape, bool trigger = false);

			void RemoveCollisionObject(btCollisionObject *obj);

			static void BulletInternalTickCallback(btDynamicsWorld * const pWorld, const btScalar timeStep);
		private:
			void HandleCallback();
			void SetupSystems();
			void CleanUpRigidBodies();
			void CleanUpSystems();
			void HandleNewCollisions(CollisionPairs& currentTickCollisions);
			btMotionState *GetMotionStateFrom(std::shared_ptr<WorldTransformComponent> transform);
		};
	}
}

#endif