#pragma once

namespace GameEngine
{
	class ITimer
	{
	public:
		virtual ~ITimer() { }
		virtual unsigned int GetTimeMs() = 0;
	};
}