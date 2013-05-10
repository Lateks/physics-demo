#include "Vec3.h"
#include <iostream>

namespace GameEngine
{
	bool Vec3::operator==(const Vec3& other) const
	{
		return Eigen::Vector3f::operator==(other) &&
			m_handedness == other.m_handedness;
	}

	std::wostream& operator<<(std::wostream& stream, const Vec3& vec)
	{
		stream << L"("
				<< vec.x() << L" "
				<< vec.y() << L" "
				<< vec.z() << L") "
				<< vec.GetHandedness();
		return stream;
	}

	std::wostream& operator<<(std::wostream& stream, const CSHandedness& handedness)
	{
		switch(handedness)
		{
		case CSHandedness::LEFT:
			stream << L"LH";
			break;
		case CSHandedness::RIGHT:
			stream << L"RH";
			break;
		case CSHandedness::NONE:
			stream << L"NH";
			break;
		}
		return stream;
	}
}