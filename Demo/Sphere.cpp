#include "stdafx.h"
#include "Sphere.h"
#include <irrlicht.h>

namespace GameEngine
{
	Sphere::Sphere(float radius, irr::scene::ISceneManager *scene)
		: GameActor(scene->addSphereSceneNode(radius)), _radius(radius) { }

	Sphere::~Sphere() { }
}