#include "SphereShape.h"
#include "LinearAlgebraTypes.h"

Point3D SphereShape::GetFarthestPointInDirection(const Vector3D& direction) const
{
	Vector3D nd = direction.normalize();
	Point3D location = _gameWorldObject->GetWorldLocation();
	return location + _radius * nd;
}