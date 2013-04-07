#ifndef GAME_H
#define GAME_H

struct GameImpl;

class Game
{
public:
	Game();
	~Game();
	Game(Game& game);
	int Run();
private:
	GameImpl *pImpl;
};

#endif