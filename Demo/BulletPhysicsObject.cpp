#include "BulletPhysicsObject.h"

namespace GameEngine
{
	namespace Physics
	{
		ConstraintID BulletPhysicsObject::constraintId = 0;

		ConstraintID BulletPhysicsObject::AddConstraint(btTypedConstraint *pConstraint)
		{
			if (IsDynamic())
			{
				ConstraintID id = ++constraintId;
				m_pConstraints[id] = pConstraint;
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
				delete it->second;
				m_pConstraints.erase(it);
			}
		}

		btTypedConstraint *BulletPhysicsObject::GetConstraint(ConstraintID id)
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