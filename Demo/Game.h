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
		bool Initialize();
		void Shutdown();
	};
}