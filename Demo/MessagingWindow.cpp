#include "stdafx.h"
#include "MessagingWindow.h"
#include "MessagingWindowImpl.h"
#include <irrlicht.h>
#include <string>
#include <iostream>
#include <cassert>

namespace 
{
	const unsigned int LINE_HEIGHT = 15;
}

namespace GameEngine
{
	namespace Display
	{

		MessagingWindow::MessagingWindow(unsigned int width, unsigned int height)
		{
			pImpl = new MessagingWindowImpl();
			pImpl->winHeight = height;
			pImpl->winWidth = width;
			pImpl->maxMessages = height/LINE_HEIGHT;
			pImpl->color = irr::video::SColor(255,255,255,255);
		}

		MessagingWindow::~MessagingWindow()
		{
			delete pImpl;
		}

		void MessagingWindow::AddMessage(const irr::core::stringw message)
		{
			if (pImpl->messageBuffer.size() == pImpl->maxMessages)
			{
				pImpl->messageBuffer.erase(pImpl->messageBuffer.begin());
			}
			pImpl->messageBuffer.push_back(message);
		}

		void MessagingWindow::SetPosition(unsigned int minX, unsigned int minY)
		{
			pImpl->posX = minX;
			pImpl->posY = minY;
		}

		void MessagingWindow::SetFont(irr::gui::IGUIFont *font)
		{
			pImpl->font = font;
		}

		void MessagingWindow::Render()
		{
			assert(pImpl->font != nullptr);
			unsigned int x = pImpl->posX;
			unsigned int y = pImpl->posY;
			for (std::size_t i = 0; i < pImpl->messageBuffer.size(); i++)
			{
				pImpl->font->draw(
					pImpl->messageBuffer[i],
					irr::core::rect<irr::s32>(x, y, x + pImpl->winWidth, y + LINE_HEIGHT),
					pImpl->color);
				y += LINE_HEIGHT;
			}
		}
	}
}