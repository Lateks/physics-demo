#ifndef IRRLICHT_MESSAGING_WINDOW_H
#define IRRLICHT_MESSAGING_WINDOW_H

#include "MessagingWindow.h"
#include <irrlicht.h>
#include <string>
#include <vector>

namespace GameEngine
{
	namespace Display
	{
		class IrrlichtMessagingWindow : public MessagingWindow
		{
		public:
			IrrlichtMessagingWindow(std::size_t messageBufferSize, irr::gui::IGUIEnvironment *irrlichtGUI);
			~IrrlichtMessagingWindow();
			virtual void AddMessage(const std::wstring& message) override;
			virtual void SetPosition(unsigned int minX, unsigned int minY) override;
			virtual void SetWidth(unsigned int width) override;
			virtual void SetFont(const std::string& fontFileName) override;
			virtual void SetVisible(bool visible) override;
			virtual void Render() const override;
		private:
			irr::gui::IGUIFont *m_pFont;
			irr::gui::IGUIEnvironment *m_irrlichtGUI;
			bool m_visible;
			unsigned int m_posX;
			unsigned int m_posY;
			unsigned int m_width;
			unsigned int m_fontHeight;
			unsigned int m_maxMessages;
			std::vector<irr::core::stringw> m_messageBuffer;
			irr::video::SColor m_color;
			void SetFontHeight();
		};
	}
}

#endif