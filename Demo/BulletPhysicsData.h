#ifndef BULLET_PHYSICS_DATA_H
#define BULLET_PHYSICS_DATA_H

#include "enginefwd.h"
#include "Mat4.h"
#include "Vec3.h"
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

			// Store the rigid bodies related to game actors.
			std::map<ActorID, const btRigidBody*> m_actorToRigidBodyMap;
			std::map<const btRigidBody*, ActorID> m_rigidBodyToActorMap;
			btRigidBody *GetRigidBody(ActorID id) const;
			ActorID GetActorID(btRigidBody const *pBody) const;

			CollisionPairs m_PreviousTickCollisions;

			void SendNewCollisionEvent(const btPersistentManifold * manifold,
				const btRigidBody * pBody1, const btRigidBody * pBody2);
			void SendSeparationEvent(const btRigidBody * pBody1, const btRigidBody * pBody2);

			void AddShape(StrongActorPtr pActor, btCollisionShape *shape,
				float mass, const std::string& physicsMaterial);

			void RemoveCollisionObject(btCollisionObject *obj);

			static void BulletInternalTickCallback(btDynamicsWorld * const pWorld, const btScalar timeStep);
		private:
			void SetupSystems();
			void CleanUpRigidBodies();
			void CleanUpSystems();
		};

		btTransform Mat4_to_btTransform(const LinearAlgebra::Mat4& transform);
		LinearAlgebra::Mat4 btTransform_to_Mat4(const btTransform& transform);

		// A struct used to convert between Bullet's transform
		// matrices and the transform matrix used by the game engine.
		// (Like ActorMotionState in McShaffry page 597.)
		struct WorldTransformConversion : public btMotionState
		{
			LinearAlgebra::Mat4 m_transform;
			WorldTransformConversion(LinearAlgebra::Mat4& transform)
				: m_transform(transform) { }

			virtual void getWorldTransform(btTransform& trans) const
			{
				trans = Mat4_to_btTransform(m_transform);
			}

			virtual void setWorldTransform(btTransform& trans)
			{
				m_transform = btTransform_to_Mat4(trans);
			}
		};
	}
}

#endif