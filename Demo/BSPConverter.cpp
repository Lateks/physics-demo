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

(Note: Modified by removing the unused parts from this file.)
*/

#include "BspConverter.h"
#include "BspLoader.h"
#include "LinearMath/btVector3.h"
#include "LinearMath/btGeometryUtil.h"
#include <string.h>


void BspConverter::convertBsp(BspLoader& bspLoader,float scaling)
{
	for (int i=0;i<bspLoader.m_numleafs;i++)
	{			
		bool isValidBrush = false;
			
		BSPLeaf&	leaf = bspLoader.m_dleafs[i];
	
		for (int b=0;b<leaf.numLeafBrushes;b++)
		{
			btAlignedObjectArray<btVector3> planeEquations;
				
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
						btVector3 planeEq;
						planeEq.setValue(
							plane.normal[0],
							plane.normal[1],
							plane.normal[2]);
						planeEq[3] = scaling*-plane.dist;

						planeEquations.push_back(planeEq);
						isValidBrush=true;
					}
					if (isValidBrush)
					{
						btAlignedObjectArray<btVector3>	vertices;
						btGeometryUtil::getVerticesFromPlaneEquations(planeEquations,vertices);
						addConvexVerticesCollider(vertices);
					}
				}
			} 
		}
	}
}