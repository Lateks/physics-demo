#include "Vec3.h"
#include "Vec3Impl.h"
#include <Eigen/Dense>

namespace GameEngine
{
	namespace LinearAlgebra
	{
		Vec3::Vec3(double x, double y, double z)
		{
			pImpl = new Vec3Impl(x, y, z);
		}

		Vec3::Vec3()
		{
			pImpl = new Vec3Impl(0, 0, 0);
		}

		Vec3::Vec3(const Vec3& other)
		{
			pImpl = new Vec3Impl(other.pImpl->value);
		}

		Vec3::Vec3(Vec3&& other)
		{
			if (this != &other)
			{
				this->pImpl = other.pImpl;
				other.pImpl = nullptr;
			}
		}

		Vec3::~Vec3()
		{
			delete pImpl;
		}

		bool Vec3::operator==(const Vec3& other) const
		{
			return pImpl->value == other.pImpl->value;
		}

		Vec3 Vec3::operator=(const Vec3& other)
		{
			if (this != &other)
			{
				pImpl->value = other.pImpl->value;
			}
			return *this;
		}

		Vec3 Vec3::operator=(Vec3&& other)
		{
			if (this != &other)
			{
				delete pImpl;
				pImpl = other.pImpl;
				other.pImpl = nullptr;
			}
			return *this;
		}

		Vec3 Vec3::operator+(const Vec3& other) const
		{
			Eigen::Vector3d result = pImpl->value + other.pImpl->value;
			return Vec3(result.x(), result.y(), result.z());
		}

		Vec3 Vec3::operator-(const Vec3& other) const
		{
			Eigen::Vector3d result = pImpl->value + other.pImpl->value;
			return Vec3(result.x(), result.y(), result.z());
		}

		Vec3 Vec3::cross(const Vec3& other) const
		{
			Eigen::Vector3d result = pImpl->value.cross(other.pImpl->value);
			return Vec3(result.x(), result.y(), result.z());
		}

		double Vec3::dot(const Vec3& other) const
		{
			return pImpl->value.dot(other.pImpl->value);
		}

		Vec3 Vec3::normalize() const
		{
			Eigen::Vector3d result = pImpl->value.normalized();
			return Vec3(result.x(), result.y(), result.z());
		}

		double Vec3::getX() const
		{
			return pImpl->value.x();
		}

		double Vec3::getY() const
		{
			return pImpl->value.y();
		}

		double Vec3::getZ() const
		{
			return pImpl->value.z();
		}

		Vec3 Vec3::operator*(double scalar) const
		{
			Eigen::Vector3d result = pImpl->value * scalar;
			return Vec3(result.x(), result.y(), result.z());
		}

		Vec3 operator*(double scalar, const Vec3& vec)
		{
			return vec * scalar;
		}

		Vec3 Vec3::operator-() const
		{
			Eigen::Vector3d result = -pImpl->value;
			return Vec3(result.x(), result.y(), result.z());
		}

		std::ostream& operator<<(std::ostream& stream, const Vec3& vec)
		{
			stream << "("
				   << vec.getX() << " "
				   << vec.getY() << " "
				   << vec.getZ() << ")";
			return stream;
		}
	}
}