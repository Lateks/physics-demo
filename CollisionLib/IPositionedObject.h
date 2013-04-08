#ifndef I_POSITIONED_OBJECT_H
#define I_POSITIONED_OBJECT_H

struct Vec3d
{
	double X;
	double Y;
	double Z;
};

class IPositionedObject
{
public:
	virtual ~IPositionedObject();
	virtual Vec3d GetWorldLocation() = 0;
protected:
	IPositionedObject();
};

#endif