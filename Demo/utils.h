#ifndef UTILS_H
#define UTILS_H

/* Sets the deleted pointer to nullptr to avoid
 * undefined behaviour in case the deleted pointer
 * is dereferenced.
 */
template <typename T>
void safe_delete(T*& obj)
{
	delete obj;
	obj = nullptr;
}

#endif