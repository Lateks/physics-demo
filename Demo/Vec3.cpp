#include "Vec3.h"
#include <iostream>

namespace GameEngine
{
	std::wostream& operator<<(std::wostream& stream, const Vec3& vec)
	{
		stream << L"("
				<< vec.x() << L" "
				<< vec.y() << L" "
				<< vec.z() << L") ";
		return stream;
	}
}