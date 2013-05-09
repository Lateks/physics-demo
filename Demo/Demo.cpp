// Demo.cpp : Defines the entry point for the console application.
//

#include "Game.h"

int main()
{
	Demo::Game game;

	if (!game.Run())
		return 1;
	return 0;
}

