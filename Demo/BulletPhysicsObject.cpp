#include "BulletPhysicsObject.h"

namespace GameEngine
{
	namespace Physics
	{
		BulletPhysicsConstraint::~BulletPhysicsConstraint()
		{
			delete m_pConstraint;
		}

		BulletPhysicsConstraint::BulletPhysicsConstraint(BulletPhysicsConstraint&& other)
		{
			if (this != &other)
			{
				this->m_pConstraint = other.m_pConstraint;
				other.m_pConstraint = nullptr;

				this->m_pConstraintUpdater = other.m_pConstraintUpdater;
				other.m_pConstraintUpdater.reset();

				this->m_updaterEventType = other.m_updaterEventType;
			}
		}

		BulletPhysicsConstraint& BulletPhysicsConstraint::operator=(BulletPhysicsConstraint&& other)
		{
			if (this != &other)
			{
				this->m_pConstraint = other.m_pConstraint;
				other.m_pConstraint = nullptr;

				this->m_pConstraintUpdater = other.m_pConstraintUpdater;
				other.m_pConstraintUpdater.reset();

				this->m_updaterEventType = other.m_updaterEventType;
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