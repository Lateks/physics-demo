#pragma once
#include <string>
#include <LinearAlgebraTypes.h>
#include "CppUnitTest.h"
#include "CppUnitTestAssert.h"

namespace Microsoft
{
	namespace VisualStudio
	{
		namespace CppUnitTestFramework
		{
			template <> static std::wstring ToString<Vector3D>(const Vector3D& vec)
			{
				RETURN_WIDE_STRING(&vec);
			}
		}
	}
}