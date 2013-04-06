// Demo.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"
#include "BasicIrrlichtRenderer.h"
#include <irrlicht.h>
#include <memory>
#include <iostream>

int main()
{
	BasicIrrlichtRenderer renderer;

	if (!renderer.SetupAndOpenWindow(800, 600, irr::video::EDT_OPENGL))
	{
		std::cout << "Failed to open device." << std::endl;
		return 1;
	}

	while (renderer.IsRunning())
	{
		renderer.UpdateScene();
	}

	return 0;
}

