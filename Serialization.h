#ifndef _SERIALIZATION_H_
#define _SERIALIZATION_H_

#include <fstream>

/// <description>
/// TODO: a temporary serialization base class.
/// </description>
class Serialization
{
public:
	/// <description>
	/// Serialize.
	/// </description>
	virtual bool Serialize(const void* data, std::ofstream& fout) = 0;

	/// <description>
	/// Deserialize.
	/// </description>
	virtual bool Deserialize(void* data, std::ifstream& fin) = 0;
};

#endif