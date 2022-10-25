#ifndef _ARRAY_H_
#define _ARRAY_H_

#include <vector>

/// <description>
/// TODO: define own array class.
/// </description>
template <typename T>
class Array : public std::vector<T> 
{
public:
	/// <description>
	/// Define insert method which accept the index parameter.
	/// </description>	
	void insert(int index, const T& value)
	{
		std::vector<T>::insert(begin() + index, value);
	}
};

#endif