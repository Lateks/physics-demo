#ifndef SPHERE_H
#define SPHERE_H

#include "GameActor.h"

namespace irr
{
	namespace scene
	{
		class ISceneManager;
	}
}

class Sphere : public GameActor
{
public:
	Sphere(float radius, irr::scene::ISceneManager *scene);
	virtual ~Sphere();
	float GetRadius() { return _radius; }
private:
	float _radius;
};

#endif