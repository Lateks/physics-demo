#include "BspLoaderFactory.h"
#include <fstream>
#include <string>
#include <iostream>

namespace GameEngine
{
	struct raw_pointer_deleter
	{
		void operator()(void* ptr) { free(ptr); }
	};

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
			std::size_t fileSize = bspFile.tellg();
			bspFile.seekg(0, std::ios::beg);

			std::unique_ptr<void, raw_pointer_deleter> memoryBuffer(malloc(fileSize + 1));
			if (bspFile.is_open())
			{
				bspFile.read((char*) memoryBuffer.get(), fileSize);
			}
			bspLoader.reset(new BspLoader());
			bspLoader->loadBSPFile(memoryBuffer.get());
		}
		bspFile.close();
		return bspLoader;
	}
}