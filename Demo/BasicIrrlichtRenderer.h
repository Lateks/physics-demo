#ifndef BASIC_IRRLICHT_RENDERER_H
#define BASIC_IRRLICHT_RENDERER_H

// Forward declarations for the referenced names in the irr namespace.
namespace irr
{
	namespace video
	{
		enum E_DRIVER_TYPE;
	}
}

struct BasicIrrlichtRendererImpl;

class BasicIrrlichtRenderer
{
public:
	BasicIrrlichtRenderer();
	~BasicIrrlichtRenderer();
	bool SetupAndOpenWindow(unsigned int width, unsigned int height, irr::video::E_DRIVER_TYPE driverType);
	bool IsRunning();
	void UpdateScene();
private:
	BasicIrrlichtRendererImpl *pImpl;
};

#endif