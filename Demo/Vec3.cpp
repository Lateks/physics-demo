#include "Vec3.h"
#include <iostream>

namespace GameEngine
{
	std::ostream& operator<<(std::ostream& stream, const Vec3& vec)
	{
		stream << "("
				<< vec.x() << " "
				<< vec.y() << " "
				<< vec.z() << ")";
		return stream;
	}
}