#include "BulletPhysicsObject.h"
#include "BulletPhysicsConstraint.h"
#include <btBulletCollisionCommon.h>
#include <btBulletDynamicsCommon.h>
#include <iostream>

namespace GameEngine
{
	namespace Physics
	{
		ConstraintID BulletPhysicsObject::constraintId = 0;

		ConstraintID BulletPhysicsObject::AddConstraint(btTypedConstraint *pConstraint,
			BulletPhysicsConstraint::ConstraintType constraintType)
		{
			if (IsDynamic())
			{
				std::shared_ptr<BulletPhysicsConstraint> constraint;
				switch (constraintType)
				{
				case BulletPhysicsConstraint::ConstraintType::BASIC_CONSTRAINT:
					constraint.reset(new BulletPhysicsConstraint(pConstraint));
					break;
				case BulletPhysicsConstraint::ConstraintType::PICK_CONSTRAINT:
					constraint.reset(new BulletPickConstraint(pConstraint));
					break;
				default:
					std::cerr << "Unknown constraint type." << std::endl;
					return 0;
				}
				ConstraintID id = ++constraintId;
				m_pConstraints[id] = constraint;
				return id;
			}
			else
			{
				return 0;
			}
		}

		void BulletPhysicsObject::RemoveConstraint(ConstraintID id)
		{
			auto it = m_pConstraints.find(id);
			if (it != m_pConstraints.end())
			{
				m_pConstraints.erase(it);
			}
		}

		std::shared_ptr<BulletPhysicsConstraint> BulletPhysicsObject::GetConstraint(ConstraintID id)
		{
			auto it = m_pConstraints.find(id);
			if (it != m_pConstraints.end())
			{
				return it->second;
			}
			return nullptr;
		}
	}
}