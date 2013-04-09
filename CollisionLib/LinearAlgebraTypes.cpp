#include "LinearAlgebraTypes.h"
#include "Vector3DImpl.h"
#include <Eigen/Dense>

Vector3D::Vector3D(double x, double y, double z)
{
	pImpl = new Vector3DImpl(x, y, z);
}

Vector3D::Vector3D()
{
	pImpl = new Vector3DImpl(0, 0, 0);
}

Vector3D::Vector3D(const Vector3D& other)
{
	pImpl = new Vector3DImpl(other.pImpl->value);
}

Vector3D::Vector3D(Vector3D&& other)
{
	if (this != &other)
	{
		this->pImpl = other.pImpl;
		other.pImpl = nullptr;
	}
}

Vector3D::~Vector3D()
{
	delete pImpl;
}

bool Vector3D::operator==(const Vector3D& other) const
{
	return pImpl->value == other.pImpl->value;
}

Vector3D Vector3D::operator=(const Vector3D& other)
{
	if (this != &other)
	{
		pImpl->value = other.pImpl->value;
	}
	return *this;
}

Vector3D Vector3D::operator=(Vector3D&& other)
{
	if (this != &other)
	{
		delete pImpl;
		pImpl = other.pImpl;
		other.pImpl = nullptr;
	}
	return *this;
}

Vector3D Vector3D::operator+(const Vector3D& other) const
{
	Eigen::Vector3d result = pImpl->value + other.pImpl->value;
	return Vector3D(result.x(), result.y(), result.z());
}

Vector3D Vector3D::operator-(const Vector3D& other) const
{
	Eigen::Vector3d result = pImpl->value + other.pImpl->value;
	return Vector3D(result.x(), result.y(), result.z());
}

Vector3D Vector3D::cross(const Vector3D& other) const
{
	Eigen::Vector3d result = pImpl->value.cross(other.pImpl->value);
	return Vector3D(result.x(), result.y(), result.z());
}

double Vector3D::dot(const Vector3D& other) const
{
	return pImpl->value.dot(other.pImpl->value);
}

Vector3D Vector3D::normalize() const
{
	Eigen::Vector3d result = pImpl->value.normalized();
	return Vector3D(result.x(), result.y(), result.z());
}

double Vector3D::getX() const
{
	return pImpl->value.x();
}

double Vector3D::getY() const
{
	return pImpl->value.y();
}

double Vector3D::getZ() const
{
	return pImpl->value.z();
}

Vector3D Vector3D::operator*(double scalar) const
{
	Eigen::Vector3d result = pImpl->value * scalar;
	return Vector3D(result.x(), result.y(), result.z());
}

Vector3D operator*(double scalar, const Vector3D& vec)
{
	return vec * scalar;
}

Vector3D Vector3D::operator-() const
{
	Eigen::Vector3d result = -pImpl->value;
	return Vector3D(result.x(), result.y(), result.z());
}

std::ostream& operator<<(std::ostream& stream, const Vector3D& vec)
{
	stream << "("
		   << vec.getX() << " "
		   << vec.getY() << " "
		   << vec.getZ() << ")";
	return stream;
}