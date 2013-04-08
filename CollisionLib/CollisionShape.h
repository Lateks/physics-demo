#ifndef COLLISION_SHAPE_H
#define COLLISION_SHAPE_H
#include "LinearAlgebraTypes.h"

class ICollisionShape
{
public:
	virtual ~ICollisionShape() { };
	virtual Point3D GetFarthestPointInDirection(const Vector3D& direction) const = 0;
protected:
	ICollisionShape() { }
};

#endif