#ifndef BULLET_PHYSICS_CONSTRAINT_H
#define BULLET_PHYSICS_CONSTRAINT_H

#include "enginefwd.h"
#include "IEventManager.h"
#include "Vec3.h"
#include <btBulletDynamicsCommon.h>

namespace GameEngine
{
	namespace Physics
	{
		class BulletPhysicsConstraint
		{
		public:
			enum class ConstraintType
			{
				BASIC_CONSTRAINT,
				PICK_CONSTRAINT
			};

			BulletPhysicsConstraint(btTypedConstraint *constraint)
				: m_pConstraint(constraint), m_updaterEventType(Events::EventType::NONE) { }
			BulletPhysicsConstraint(btTypedConstraint *constraint, Events::EventHandlerPtr handler, Events::EventType type)
				: m_pConstraint(constraint), m_pConstraintUpdater(handler), m_updaterEventType(type) { }
			virtual ~BulletPhysicsConstraint()
			{
				delete m_pConstraint;
			}

			Events::EventHandlerPtr GetConstraintUpdater()
			{
				return m_pConstraintUpdater;
			}
			Events::EventType GetHandlerEventType()
			{
				return m_updaterEventType;
			}
			btTypedConstraint *GetBulletConstraint()
			{
				return m_pConstraint;
			}

			// This could actually have several event types instead of just one.
			void SetConstraintUpdater(Events::EventHandlerPtr updater, Events::EventType eventType)
			{
				m_pConstraintUpdater = updater;
				m_updaterEventType = eventType;
			}

			virtual ConstraintType GetConstraintType()
			{
				return constraintType;
			}
		private:
			const static ConstraintType constraintType = ConstraintType::BASIC_CONSTRAINT;
			BulletPhysicsConstraint(BulletPhysicsConstraint& other);
			BulletPhysicsConstraint& operator=(BulletPhysicsConstraint& other);
			btTypedConstraint *m_pConstraint;
			Events::EventHandlerPtr m_pConstraintUpdater;
			Events::EventType m_updaterEventType;
		};

		class BulletPickConstraint : public BulletPhysicsConstraint
		{
		public:
			BulletPickConstraint(btTypedConstraint *constraint)
				: BulletPhysicsConstraint(constraint) { }
			BulletPickConstraint(btTypedConstraint *constraint, float pickDistance)
				: BulletPhysicsConstraint(constraint), m_pickDistance(pickDistance) { }
			BulletPickConstraint(btTypedConstraint *constraint, Events::EventHandlerPtr handler,
				Events::EventType type, float pickDistance)
				: BulletPhysicsConstraint(constraint, handler, type), m_pickDistance(pickDistance) { }
			virtual ~BulletPickConstraint() { }

			float GetPickDistance()
			{
				return m_pickDistance;
			}
			void SetPickDistance(float pickDistance)
			{
				m_pickDistance = pickDistance;
			}
			Vec3 GetOriginalAngularFactor()
			{
				return m_angularFactor;
			}
			void SetOriginalAngularFactor(Vec3& angularFactor)
			{
				m_angularFactor = angularFactor;
			}

			virtual ConstraintType GetConstraintType()
			{
				return constraintType;
			}
		private:
			BulletPickConstraint(BulletPickConstraint& other);
			BulletPickConstraint& operator=(BulletPickConstraint& other);

			Vec3 m_angularFactor;
			float m_pickDistance;
			const static ConstraintType constraintType = ConstraintType::PICK_CONSTRAINT;
		};
	}
}

#endif