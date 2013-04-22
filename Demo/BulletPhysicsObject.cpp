#include "BulletPhysicsObject.h"

namespace GameEngine
{
	namespace Physics
	{
		BulletPhysicsConstraint::~BulletPhysicsConstraint()
		{
			delete pConstraint;
		}

		BulletPhysicsConstraint::BulletPhysicsConstraint(BulletPhysicsConstraint&& other)
		{
			if (this != &other)
			{
				this->pConstraint = other.pConstraint;
				other.pConstraint = nullptr;

				this->pConstraintUpdater = other.pConstraintUpdater;
				other.pConstraintUpdater.reset();

				this->updaterEventType = other.updaterEventType;
			}
		}

		BulletPhysicsConstraint& BulletPhysicsConstraint::operator=(BulletPhysicsConstraint&& other)
		{
			if (this != &other)
			{
				this->pConstraint = other.pConstraint;
				other.pConstraint = nullptr;

				this->pConstraintUpdater = other.pConstraintUpdater;
				other.pConstraintUpdater.reset();

				this->updaterEventType = other.updaterEventType;
			}
			return *this;
		}

		ConstraintID BulletPhysicsObject::constraintId = 0;

		ConstraintID BulletPhysicsObject::AddConstraint(btTypedConstraint *pConstraint)
		{
			if (IsDynamic())
			{
				ConstraintID id = ++constraintId;
				m_pConstraints[id] = std::shared_ptr<BulletPhysicsConstraint>(new BulletPhysicsConstraint(pConstraint));
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