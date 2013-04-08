#ifndef COLLISION_MATH_H
#define COLLISION_MATH_H
#include "CollisionShape.h"

bool GJKCollide(const ICollisionShape& shape1, const ICollisionShape& shape2);

#endif