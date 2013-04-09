#include "Cube.h"
#include <irrlicht.h>

namespace GameEngine
{
	Cube::Cube(float edgeLength, irr::scene::ISceneManager *scene)
		: GameActor(scene->addCubeSceneNode(edgeLength)), _edgeLength(edgeLength) {}

	Cube::~Cube() {}
}