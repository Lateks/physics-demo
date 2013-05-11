#include "CppUnitTest.h"
#include <Vec3.h>
#include <Vec4.h>

namespace Microsoft
{
	namespace VisualStudio
	{
		namespace CppUnitTestFramework
		{
			template <> static std::wstring ToString<GameEngine::Vec3>(const GameEngine::Vec3& t) { RETURN_WIDE_STRING(t); }
			template <> static std::wstring ToString<GameEngine::Vec4>(const GameEngine::Vec4& t) { RETURN_WIDE_STRING(t); }
			template <> static std::wstring ToString<GameEngine::CSHandedness>(const GameEngine::CSHandedness& t) { RETURN_WIDE_STRING(t); }
		}
	}
}