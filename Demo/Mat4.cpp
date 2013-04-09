#include "Mat4.h"
#include "Mat4Impl.h"
#include <Eigen/Dense>

namespace GameEngine
{
	namespace LinearAlgebra
	{
		Mat4::Mat4()
		{
			pImpl = new Mat4Impl(Eigen::Matrix4d::Identity());
		}

		Mat4::Mat4(double val)
		{
			pImpl = new Mat4Impl(Eigen::Matrix4d::Constant(val));
		}

		Mat4::Mat4(const Mat4& other)
		{
			if (this != &other)
			{
				delete pImpl;
				pImpl = other.pImpl;
			}
		}

		Mat4::Mat4(Mat4&& other)
		{
			if (this != &other)
			{
				delete pImpl;
				pImpl = other.pImpl;
				other.pImpl = nullptr;
			}
		}

		Mat4::~Mat4()
		{
			delete pImpl;
		}

		Mat4 Mat4::operator=(const Mat4& other)
		{
			if (this != &other)
			{
				delete pImpl;
				pImpl = other.pImpl;
			}
			return *this;
		}

		Mat4 Mat4::operator=(Mat4&& other)
		{
			if (this != &other)
			{
				delete pImpl;
				pImpl = other.pImpl;
				other.pImpl = nullptr;
			}
			return *this;
		}

		bool Mat4::operator==(const Mat4& other) const
		{
			return pImpl->value == other.pImpl->value;
		}

		Mat4 Mat4::operator+(const Mat4& other) const
		{
			Mat4 matResult;
			matResult.pImpl->value = pImpl->value + other.pImpl->value;
			return matResult;
		}

		Mat4 Mat4::operator-(const Mat4& other) const
		{
			Mat4 matResult;
			matResult.pImpl->value = pImpl->value - other.pImpl->value;
			return matResult;
		}

		Mat4 Mat4::operator*(double scalar) const
		{
			Mat4 matResult;
			matResult.pImpl->value = scalar *pImpl->value;
			return matResult;
		}

		Mat4 Mat4::operator*(const Mat4& other) const
		{
			Mat4 matResult;
			matResult.pImpl->value = pImpl->value * other.pImpl->value;
			return matResult;
		}

		Mat4 Mat4::operator-() const
		{
			Mat4 matResult;
			matResult.pImpl->value = -pImpl->value;
			return matResult;
		}

		double& Mat4::index(const size_t i, const size_t j)
		{
			return pImpl->value(i, j);
		}

		Mat4 operator*(double scalar, const Mat4& mat)
		{
			return mat * scalar;
		}
	}
}