#ifndef COLLISION_SHAPE_H
#define COLLISION_SHAPE_H
#include <Eigen/Dense>

typedef Eigen::Vector3d Point3D;
typedef Eigen::Vector3d Vector3D;

class CollisionShape
{
public:
	CollisionShape();
	virtual ~CollisionShape();
	virtual Point3D GetFarthestPointInDirection(Vector3D direction) const = 0;
};

#endif