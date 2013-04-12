#ifndef I_TIMER_H
#define I_TIMER_H

namespace GameEngine
{
	class ITimer
	{
	public:
		virtual ~ITimer() { }
		virtual unsigned int GetTime() = 0;
	};
}

#endif