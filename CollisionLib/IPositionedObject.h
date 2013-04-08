#ifndef I_POSITIONED_OBJECT_H
#define I_POSITIONED_OBJECT_H
#include "LinearAlgebraTypes.h"

class IPositionedObject
{
public:
	virtual ~IPositionedObject() { };
	virtual Vector3D GetWorldLocation() = 0;
protected:
	IPositionedObject() { };
};

#endif