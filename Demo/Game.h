#pragma once

namespace GameEngine
{
	class Game
	{
	public:
		Game();
		virtual ~Game();
		bool Run();
	private:
		bool Setup();
	};
}