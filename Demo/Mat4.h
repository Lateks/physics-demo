#ifndef MAT_4_H
#define MAT_4_H

#include "enginefwd.h"

namespace GameEngine
{
	namespace LinearAlgebra
	{
		class Mat4
		{
		public:
			Mat4(); // creates an identity matrix
			Mat4(double val); // creates a matrix where all values are val
			Mat4(const Mat4& other);
			Mat4(Mat4&& other);
			~Mat4();
			Mat4 operator=(const Mat4& other);
			Mat4 operator=(Mat4&& other);
			bool operator==(const Mat4& other) const;
			Mat4 operator+(const Mat4& other) const;
			Mat4 operator-(const Mat4& other) const;
			Mat4 operator*(double scalar) const;
			Mat4 operator*(const Mat4& other) const;
			Mat4 operator-() const;
			double& index(const size_t row, const size_t col);
			double index(const size_t row, const size_t col) const;
		private:
			Mat4Impl *pImpl;
		};

		Mat4 operator*(double scalar, const Mat4& mat);
	}
}

#endif