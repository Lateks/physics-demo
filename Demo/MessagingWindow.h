#ifndef MESSAGING_WINDOW_H
#define MESSAGING_WINDOW_H

#include "enginefwd.h"
#include <string>

namespace GameEngine
{
	namespace Display
	{
		class MessagingWindow
		{
		public:
			~MessagingWindow() { };
			virtual void AddMessage(const std::wstring& message) = 0;
			virtual void SetPosition(unsigned int minX, unsigned int minY) = 0;
			virtual void SetWidth(unsigned int width) = 0;
			virtual void SetFont(const std::string& fontFileName) = 0;
			virtual void SetVisible(bool visible) = 0;
			virtual void Render() const = 0;
		};
	}
}

#endif