#ifndef LINEAR_ALGEBRA_TYPES_H
#define LINEAR_ALGEBRA_TYPES_H
#include <string>

struct Vector3DImpl;

class Vector3D
{
public:
	Vector3D();
	Vector3D(double x, double y, double z);
	Vector3D(const Vector3D& other);
	Vector3D(Vector3D&& other);
	~Vector3D();
	Vector3D operator=(const Vector3D& other);
	Vector3D operator=(Vector3D&& other);
	bool operator==(const Vector3D& other) const;
	Vector3D operator+(const Vector3D& other) const;
	Vector3D operator-(const Vector3D& other) const;
	Vector3D operator*(double scalar) const;
	Vector3D operator-() const;
	Vector3D cross(const Vector3D& other) const;
	double dot(const Vector3D& other) const;
	Vector3D normalize() const;
	double getX() const;
	double getY() const;
	double getZ() const;
private:
	Vector3DImpl *pImpl;
};

Vector3D operator*(double scalar, const Vector3D& vec);
std::ostream& operator<<(std::ostream& stream, const Vector3D& vec);

typedef Vector3D Point3D;
typedef Vector3D Plane3D; // as defined by the plane normal

#endif