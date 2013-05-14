// Demo.cpp : Defines the entry point for the console application.
//

#include "IrrlichtDisplay.h"
#include "BulletPhysics.h"
#include "DemoGameLogic.h"
#include "Game.h"

using GameEngine::Display::IrrlichtDisplayFactory;
using Demo::DemoGameLogicFactory;
using GameEngine::Physics::BulletPhysicsFactory;

int main()
{
	GameEngine::Game game;
	game.Initialize(DemoGameLogicFactory(),
		IrrlichtDisplayFactory(1024, 800, GameEngine::Display::DriverType::OPEN_GL, GameEngine::Display::CameraType::FPS_WASD),
		BulletPhysicsFactory("..\\assets\\materials.xml", 0.05f));

	if (!game.Run())
		return 1;
	return 0;
}

