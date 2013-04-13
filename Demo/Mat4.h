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
			Mat4(); // creates a zero matrix
			Mat4(const Mat4& other);
			Mat4(Mat4&& other);
			~Mat4();
			Mat4 operator=(const Mat4& other);
			Mat4 operator=(Mat4&& other);
			bool operator==(const Mat4& other) const;
			bool operator!=(const Mat4& other) const;
			float& index(const size_t row, const size_t col);
			float index(const size_t row, const size_t col) const;
		private:
			Mat4Impl *pImpl;
		};
	}
}

#endif