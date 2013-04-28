#ifndef BSP_CONVERTER_H
#define BSP_CONVERTER_H

#include "Vec4.h"
#include <vector>
#include <functional>

class BspLoader;

namespace Utils
{
	// Loads the plane equations describing convex objects that
	// make up the level geometry described by a *.bsp file (loaded
	// using a BspLoader).
	void ConvertBsp(BspLoader& bspLoader,
		std::function<void(std::vector<GameEngine::Vec4>& planeEquations)> addConvexMesh);
}
#endif