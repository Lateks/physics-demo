#include "Mat4.h"
#include "Mat4Impl.h"

namespace GameEngine
{
	Mat4::Mat4()
	{
		pImpl = new Mat4Impl();
	}

	Mat4::Mat4(const Mat4& other)
	{
		if (this != &other)
		{
			*this->pImpl = *other.pImpl;
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
			*this->pImpl = *other.pImpl;
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
		for (size_t i = 0; i < 4; i++)
			for (size_t j = 0; j < 4; j++)
				if (pImpl->matrix[i][j] != other.pImpl->matrix[i][j])
					return false;
		return true;
	}

	bool Mat4::operator!=(const Mat4& other) const
	{
		return !(*this == other);
	}

	float& Mat4::index(const size_t i, const size_t j)
	{
		return pImpl->matrix[i][j];
	}

	float Mat4::index(const size_t i, const size_t j) const
	{
		return pImpl->matrix[i][j];
	}
}