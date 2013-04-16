#ifndef BSP_LOADER_FACTORIES_H
#define BSP_LOADER_FACTORIES_H

#include "BSPLoader.h"
#include <memory>
#include <string>
#include <iostream>

namespace GameEngine
{
	// Reads the bsp file into a buffer and loads it into a BSP loader.
	// The returned unique_ptr is a nullptr if file reading failed.
	std::unique_ptr<BspLoader> CreateBspLoader(const std::string& bspFilePath);
}

#endif