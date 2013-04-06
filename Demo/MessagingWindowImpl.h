#ifndef MESSAGING_WINDOW_IMPL_H
#define MESSAGING_WINDOW_IMPL_H

#include <irrlicht.h>
#include <string>
#include <vector>

struct MessagingWindowImpl
{
	irr::gui::IGUIFont *font;
	unsigned int winHeight;
	unsigned int winWidth;
	unsigned int posX;
	unsigned int posY;
	unsigned int maxMessages;
	std::vector<irr::core::stringw> messageBuffer;
	irr::video::SColor color;
};

#endif