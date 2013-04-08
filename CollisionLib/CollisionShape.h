#ifndef COLLISION_SHAPE_H
#define COLLISION_SHAPE_H
#include "IPositionedObject.h"
#include <Eigen/Dense>

typedef Eigen::Vector3d Point3D;
typedef Eigen::Vector3d Vector3D;

class ICollisionShape
{
public:
	virtual ~ICollisionShape();
	virtual Point3D GetFarthestPointInDirection(Vector3D direction) const = 0;
protected:
	ICollisionShape();
};

Eigen::Vector3d ConvertVec3D(Vec3d vector)
{
	return Eigen::Vector3d(vector.X, vector.Y, vector.Z);
}

#endif