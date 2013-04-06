#ifndef MESSAGING_WINDOW_H
#define MESSAGING_WINDOW_H

#include <string>
#include <irrlicht.h>

struct MessagingWindowImpl;

class MessagingWindow
{
public:
	MessagingWindow(unsigned int width, unsigned int height);
	~MessagingWindow();
	void AddMessage(const irr::core::stringw message);
	void SetPosition(unsigned int minX, unsigned int minY);
	void SetFont(irr::gui::IGUIFont *font);
	void Render();
private:
	MessagingWindowImpl *pImpl;
};

#endif