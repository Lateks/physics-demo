#include "IrrlichtMessagingWindow.h"
#include <irrlicht.h>
#include <string>
#include <iostream>
#include <cassert>

namespace GameEngine
{
	namespace Display
	{

		IrrlichtMessagingWindow::IrrlichtMessagingWindow(std::size_t messageBufferSize, irr::gui::IGUIEnvironment *irrlichtGUI)
			: m_color(255, 255, 255, 255), m_maxMessages(messageBufferSize),
			m_posX(0), m_posY(0), m_visible(false), m_pFont(nullptr), m_width(400),
			m_irrlichtGUI(irrlichtGUI)
		{
			if (m_irrlichtGUI)
			{
				m_pFont = m_irrlichtGUI->getBuiltInFont();
				SetFontHeight();
			}
		}

		void IrrlichtMessagingWindow::SetFontHeight()
		{
			if (m_pFont)
			{
				m_fontHeight = m_pFont->getDimension(L"I").Height + 2;
			}
		}

		IrrlichtMessagingWindow::~IrrlichtMessagingWindow() { }

		void IrrlichtMessagingWindow::AddMessage(const std::wstring& message)
		{
			if (m_messageBuffer.size() == m_maxMessages)
			{
				m_messageBuffer.erase(m_messageBuffer.begin());
			}
			m_messageBuffer.push_back(irr::core::stringw(message.c_str()));
		}

		void IrrlichtMessagingWindow::SetPosition(unsigned int minX, unsigned int minY)
		{
			m_posX = minX;
			m_posY = minY;
		}

		void IrrlichtMessagingWindow::SetFont(const std::string& fontFilePath)
		{
			if (m_irrlichtGUI)
			{
				m_pFont = m_irrlichtGUI->getFont(fontFilePath.c_str());
				SetFontHeight();
			}
		}

		void IrrlichtMessagingWindow::SetVisible(bool visible)
		{
			m_visible = visible;
		}

		void IrrlichtMessagingWindow::SetWidth(unsigned int width)
		{
			m_width = width;
		}

		void IrrlichtMessagingWindow::Render() const
		{
			assert(m_pFont);
			if (!m_pFont || !m_visible)
				return;

			unsigned int x = m_posX;
			unsigned int y = m_posY;
			for (std::size_t i = 0; i < m_messageBuffer.size(); i++)
			{
				m_pFont->draw(
					m_messageBuffer[i], irr::core::rect<irr::s32>(
					x, y, x + m_width, y + m_fontHeight), m_color);
				y += m_fontHeight;
			}
		}
	}
}