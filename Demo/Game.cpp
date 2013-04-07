#include "stdafx.h"
#include "Game.h"
#include "GameImpl.h"
#include "BasicIrrlichtRenderer.h"
#include <irrlicht.h>
#include <iostream>

Game::Game()
{
	BasicIrrlichtRenderer *renderer = new BasicIrrlichtRenderer();

	if (!renderer->SetupAndOpenWindow(800, 600, irr::video::EDT_OPENGL))
	{
		std::cerr << "Failed to open OpenGL device." << std::cerr;
		return;
	}

	// TODO: setup objects (actors) in the game world.
	// TODO: setup collisions

	pImpl = new GameImpl(renderer);
}

Game::~Game()
{
	delete pImpl;
}

Game::Game(Game& game)
{
	delete pImpl;
	pImpl = game.pImpl;
	game.pImpl = nullptr;
}

int Game::Run()
{
	if (pImpl->renderer == nullptr)
		return 1;

	while (pImpl->renderer->IsRunning())
	{
		// TODO: test collisions
		// TODO: move actors
		pImpl->renderer->UpdateScene();
	}

	return 0;
}