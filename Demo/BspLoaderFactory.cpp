#include "BspLoaderFactory.h"
#include <fstream>
#include <string>
#include <iostream>

namespace GameEngine
{
	std::unique_ptr<BspLoader> CreateBspLoader(const std::string& bspFilePath)
	{
		std::unique_ptr<BspLoader> bspLoader;
		std::ifstream bspFile(bspFilePath, std::ios::binary);
		if (bspFile.fail())
		{
			std::cerr << "Failed to open BSP file in path " << bspFilePath << std::endl;
		}
		else
		{
			bspFile.seekg(0, std::ios::end);
			std::size_t fileSize = (std::size_t) bspFile.tellg();
			bspFile.seekg(0, std::ios::beg);

			std::unique_ptr<char[]> memoryBuffer(new char[fileSize + 1]);
			if (memoryBuffer && bspFile.is_open())
			{
				bspFile.read(memoryBuffer.get(), fileSize);
				bspLoader.reset(new BspLoader());
				bspLoader->loadBSPFile((void*) memoryBuffer.get());
			}
		}
		bspFile.close();
		return bspLoader;
	}
}