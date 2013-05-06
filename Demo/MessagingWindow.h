#pragma once

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
			virtual void VAddMessage(const std::wstring& message) = 0;
			virtual void VSetPosition(unsigned int minX, unsigned int minY) = 0;
			virtual void VSetWidth(unsigned int width) = 0;
			virtual void VSetFont(const std::string& fontFileName) = 0;
			virtual void VSetVisible(bool visible) = 0;
			virtual void VRender() const = 0;
		};
	}
}