// Disable warnings about fopen. (Would be better to use a more
// secure method for reading files here, but this was quick to do.)
#define _CRT_SECURE_NO_WARNINGS

#include "BspLoaderFactory.h"

namespace GameEngine
{
	std::unique_ptr<BspLoader> CreateBspLoader(const std::string& bspFilePath)
	{
		std::unique_ptr<BspLoader> bspLoader;
		FILE *file = fopen(bspFilePath.c_str(), "r");
		if (file)
		{
			size_t fileSize;
			if (fseek(file, 0, SEEK_END) ||
			   (fileSize = ftell(file)) == EOF ||
			   fseek(file, 0, SEEK_SET))
			{
				std::cerr << "Failed to get file size from " << bspFilePath << "." << std::endl;
			}
			else
			{
				void *memoryBuffer = malloc(fileSize + 1);
				fread(memoryBuffer, 1, fileSize, file);

				bspLoader.reset(new BspLoader());
				bspLoader->loadBSPFile(memoryBuffer);
			}
			fclose(file);
		}
		return bspLoader;
	}
}