#include "Vec3.h"
#include <iostream>

namespace GameEngine
{
	std::wostream& operator<<(std::wostream& stream, const Vec3& vec)
	{
		stream << L"("
				<< vec.x() << L" "
				<< vec.y() << L" "
				<< vec.z() << L")";
		return stream;
	}

	std::wostream& operator<<(std::wostream& stream, const CSHandedness& handedness)
	{
		switch(handedness)
		{
		case CSHandedness::LEFT:
			stream << L"LEFT-HANDED";
			break;
		case CSHandedness::RIGHT:
			stream << L"RIGHT-HANDED";
			break;
		case CSHandedness::NONE:
			stream << L"NO HANDEDNESS";
			break;
		}
		return stream;
	}
}