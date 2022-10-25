#ifndef _BITSET_H_
#define _BITSET_H_

#include "Serialization.h"

// Forward decleration.
class BitSetSerialization;

/// <description>
/// BitSet class.
/// </desciption>
class BitSet
{
	friend class BitSetSerialization;

protected:
	unsigned int*			mArray;
	unsigned int			mArraySize;
	unsigned int			mBitCount;

protected:
	static const unsigned int BITSPERCELL = 32;
	static const unsigned int SHIFTBITS = 5;

public:
	/// <description>
	/// Constructor.
	/// </description>
	BitSet();

	/// <description>
	/// Destructor.
	/// </description>
	~BitSet();

public:
	/// <description>
	/// Initialize.
	/// </description>
	bool Initialize(unsigned int bitCount);	

	/// <description>
	/// Reset all bits to 0.
	/// </description>
	void Clear();

	/// <description>
	/// Set bit at given position 1.
	/// </description>
	void SetBit(unsigned int position);

	/// <description>
	/// Set bit at given position 0.
	/// </description>
	void ClearBit(unsigned int position);

	/// <description>
	/// Test bit value in given position, return true, if 1; else, return false.
	/// </description>
	bool CheckBit(unsigned int position) const;

	/// <description>
	/// Return number of bits which are set.
	/// </description>
	int Number() const;

protected:
	/// <description>
	/// Find which cell of array should be updated.
	/// </description>
	inline unsigned int ArrayIndex(unsigned int position) const
	{
		return position >> SHIFTBITS;
	}

	/// <description>
	/// Find which bit in one array cell should be updated.
	/// </description>
	inline unsigned int BitIndex(unsigned int position) const
	{
		return position & (BITSPERCELL - 1);
	}	
};

/// <description>
/// BitSet serialization class.
/// </description>
class BitSetSerialization : public Serialization
{
public:
	/// <description>
	/// Serialize.
	/// </description>
	virtual bool Serialize(const void* data, std::ofstream& fout);

	/// <description>
	/// Deserialize.
	/// </description>
	virtual bool Deserialize(void* data, std::ifstream& fin);
};

#endif