#ifndef CUBE_H
#define CUBE_H

#include "GameActor.h"

namespace irr
{
	namespace scene
	{
		class ISceneManager;
	}
}

class Cube : public GameActor
{
public:
	Cube(float edgeLength, irr::scene::ISceneManager *scene);
	virtual ~Cube();
	float GetEdgeLength() { return _edgeLength; }
private:
	float _edgeLength;
};

#endif