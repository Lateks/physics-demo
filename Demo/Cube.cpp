#include "stdafx.h"
#include "Cube.h"
#include <irrlicht.h>

Cube::Cube(float edgeLength, irr::scene::ISceneManager *scene)
	: GameActor(scene->addCubeSceneNode(edgeLength)), _edgeLength(edgeLength) {}

Cube::~Cube() {}