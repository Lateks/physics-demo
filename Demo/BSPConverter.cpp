/*
The basic implementation of the conversion method taken from the
Bullet Physics example project BspDemo.

Modifications made for the Game Engine Architecture project:
- Removed loading of elements (non-static geometry in bsp map file such
  as weapons etc.). This only loads the leaves which describe the convex
  polyhedra that make up the map geometry.

- Refactored Bullet-specific functionality into BulletPhysics.h/cpp
  to make it theoretically compatible with any physics system...
  Also: using generic data structures here (std::vector and my own Vec4)
  instead of Bullet's equivalents (btAlignedObjectArray and btVector3).

- Used a function callback for adding the vertices into the physics system
  instead of an inheritance mechanism like in the original code (just
  for simplicity) and make the conversion function a free function.

- In Quake III the Y and Z axes are swapped, so I also have to swap these
  in the calculated plane normals.

- Removed the scaling parameter since this is done inside the Bullet Physics
  wrapper class anyway.

The original example code can be found at
https://github.com/alanjrogers/bullet-physics/tree/master/Demos/BspDemo.

---------------------------------

Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2006 Erwin Coumans  http://continuousphysics.com/Bullet/

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

#include "BspConverter.h"
#include "BspLoader.h"
#include "Vec4.h"
#include <string.h>
#include <vector>
#include <functional>

using GameEngine::Vec4;

namespace Utils
{
	void ConvertBsp(BspLoader& bspLoader,
		std::function<void(std::vector<Vec4>& planeEquations)> addConvexMesh)
	{
		for (int i = 0; i < bspLoader.m_numleafs; i++)
		{	
			BSPLeaf& leaf = bspLoader.m_dleafs[i];
	
			for (int b = 0; b < leaf.numLeafBrushes; b++)
			{
				std::vector<Vec4> planeEquations;
				
				int brushid = bspLoader.m_dleafbrushes[leaf.firstLeafBrush+b];

				BSPBrush& brush = bspLoader.m_dbrushes[brushid];
				if (brush.shaderNum!=-1)
				{
					if (bspLoader.m_dshaders[brush.shaderNum].contentFlags & BSPCONTENTS_SOLID)
					{
						bool isValidBrush = false;
						brush.shaderNum = -1;

						for (int p=0;p<brush.numSides;p++)
						{
							int sideid = brush.firstSide+p;
							BSPBrushSide& brushside = bspLoader.m_dbrushsides[sideid];
							int planeid = brushside.planeNum;
							BSPPlane& plane = bspLoader.m_dplanes[planeid];

							Vec4 planeEq(plane.normal[0],
										 plane.normal[2],
										 plane.normal[1],
										 -plane.dist); // The last parameter defines the
													   // distance of the plane to the origin of the
													   // map.

							planeEquations.push_back(planeEq);
							isValidBrush=true;
						}
						if (isValidBrush)
						{
							addConvexMesh(planeEquations);
						}
					}
				} 
			}
		}
	}
}