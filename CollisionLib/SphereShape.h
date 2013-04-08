#ifndef SPHERE_SHAPE_H
#define SPHERE_SHAPE_H
#include "CollisionShape.h"
#include "IPositionedObject.h"

class SphereShape : public ICollisionShape
{
public:
	SphereShape(double radius, IPositionedObject *obj) 
		: _radius(radius), _gameWorldObject(obj) { }
	virtual Point3D GetFarthestPointInDirection(const Vector3D& direction) const;
private:
	double _radius;
	IPositionedObject *_gameWorldObject;
};

#endif