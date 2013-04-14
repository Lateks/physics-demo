#ifndef MAT_4_IMPL_H
#define MAT_4_IMPL_H

namespace GameEngine
{
	struct Mat4Impl
	{
		Mat4Impl() { }
		Mat4Impl(Mat4Impl& other)
		{
			for (size_t i = 0; i < 4; i++)
				for (size_t j = 0; j < 4; j++)
					matrix[i][j] = other.matrix[i][j];
		}
		Mat4Impl& operator=(const Mat4Impl& other)
		{
			if (this != &other)
			{
				for (size_t i = 0; i < 4; i++)
					for (size_t j = 0; j < 4; j++)
						matrix[i][j] = other.matrix[i][j];
			}
			return *this;
		}
		float matrix[4][4];
	};
}

#endif