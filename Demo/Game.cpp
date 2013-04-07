#include "stdafx.h"
#include "Game.h"
#include "GameImpl.h"
#include "IrrlichtRenderer.h"
#include <irrlicht.h>
#include <iostream>

Game::Game()
{
	IrrlichtRenderer *renderer = new IrrlichtRenderer();

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
	if (pImpl->pRenderer == nullptr)
		return 1;

	while (pImpl->pRenderer->pDevice->run())
	{
		if (pImpl->pRenderer->pDevice->isWindowActive())
		{
			// TODO: test collisions
			// TODO: move actors
			pImpl->pRenderer->DrawScene();
		}
		else
			pImpl->pRenderer->pDevice->yield();
	}

	return 0;
}