/*
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
---------------------------
Modifications made for the Game Engine Architecture project (Laura):
- Removed loading of elements (non-static geometry in bsp map file such
  as weapons etc.). Only load the leaves which describe the convex
  polyhedra that make up the map geometry.
- Refactored Bullet-specific functionality into BulletPhysics.h/cpp
  to make it *theoretically* compatible with any physics system...
  Also: using generic data structures here (std::vector and my own Vec3/4)
  instead of Bullet's equivalents (btVector3 and btAlignedObjectArray).
- Use a function callback for adding the vertices into the physics system
  instead of an inheritance mechanism like in the original code (just
  for simplicity).
*/

#include "BspConverter.h"
#include "BspLoader.h"
#include "Vec4.h"
#include <string.h>
#include <vector>
#include <functional>

using GameEngine::Vec4;

// Not sure what the scaling parameter stands for (?)
void BspConverter::convertBsp(BspLoader& bspLoader, float scaling,
	std::function<void(std::vector<Vec4>& planeEquations)> addConvexMesh)
{
	for (int i=0;i<bspLoader.m_numleafs;i++)
	{			
		bool isValidBrush = false; // TODO: is this really supposed to be set here and not inside the for loop below?
			
		BSPLeaf&	leaf = bspLoader.m_dleafs[i];
	
		for (int b=0;b<leaf.numLeafBrushes;b++)
		{
			std::vector<Vec4> planeEquations;
				
			int brushid = bspLoader.m_dleafbrushes[leaf.firstLeafBrush+b];

			BSPBrush& brush = bspLoader.m_dbrushes[brushid];
			if (brush.shaderNum!=-1)
			{
				if (bspLoader.m_dshaders[ brush.shaderNum ].contentFlags & BSPCONTENTS_SOLID)
				{
					brush.shaderNum = -1;

					for (int p=0;p<brush.numSides;p++)
					{
						int sideid = brush.firstSide+p;
						BSPBrushSide& brushside = bspLoader.m_dbrushsides[sideid];
						int planeid = brushside.planeNum;
						BSPPlane& plane = bspLoader.m_dplanes[planeid];
						Vec4 planeEq(plane.normal[0],
									 plane.normal[1],
									 plane.normal[2],
									 scaling*-plane.dist);

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