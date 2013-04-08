#ifndef COLLISION_MATH_H
#define COLLISION_MATH_H
#include <Eigen/Dense>

typedef Eigen::Vector3d Point3D;
typedef Eigen::Vector3d Vector3D;
typedef Eigen::Vector3d Plane3D; // as defined by the plane normal

class CollisionShape;

bool GJKCollide(const CollisionShape& shape1, const CollisionShape& shape2);

#endif