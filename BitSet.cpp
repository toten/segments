#include "Common.h"

#include "BitSet.h"
#include <cstdlib>
#include <cmath>
#include <cstring>

BitSet::BitSet() : mArray(NULL)
, mArraySize(0)
, mBitCount(0)
{
}

bool BitSet::Initialize(unsigned int bitCount)
{
	// TODO: Assert(bitCount > 0);
	mBitCount = bitCount;
	
	// Allocate memory for bits.
	mArraySize = (unsigned int)ceil(float(bitCount) / BITSPERCELL);
	mArray = new unsigned int[mArraySize];
	if (mArray == NULL)
		return false;
	
	// Set all 0.
	memset(mArray, 0, sizeof(unsigned int) * mArraySize);
	return true;
}

BitSet::~BitSet()
{
	if (mArray != NULL)
		delete [] mArray;
}

void BitSet::Clear()
{
	memset(mArray, 0, sizeof(unsigned int) * mArraySize);
}

void BitSet::SetBit(unsigned int position)
{
	// TODO: Assert(position > 0 && position < mBitCount);

	mArray[ArrayIndex(position)] |= (1 << BitIndex(position));
}

void BitSet::ClearBit(unsigned int position)
{
	// TODO: Assert(position > 0 && position < mBitCount);
	
	mArray[ArrayIndex(position)] &= ~(1 << BitIndex(position));
}

bool BitSet::CheckBit(unsigned int position) const
{
	// TODO: Assert(position > 0 && position < mBitCount);

	return ((mArray[ArrayIndex(position)] & (1 << BitIndex(position))) != 0);
}

int BitSet::Number() const
{
	int count = 0;
	for (int i = 0; i < (int)mArraySize; ++i)
	{
		unsigned int value = mArray[i];
		while (value)
		{
			// TODO: need more test.
			value &= value - 1;
			count++;
		}
	}

	return count;
}

bool BitSetSerialization::Serialize(const void* data, std::ofstream& fout)
{
	// Get the BitSet pointer.
	// TODO: check the validation.
	const BitSet* bitSet = (const BitSet*)data;

	// Write to the file.
	fout.write((const char*)&bitSet->mArraySize, sizeof(unsigned int));
	fout.write((const char*)bitSet->mArray, sizeof(unsigned int) * bitSet->mArraySize);
	fout.write((const char*)&bitSet->mBitCount, sizeof(unsigned int));	

	return true;
}

bool BitSetSerialization::Deserialize(void* data, std::ifstream& fin)
{
	// Get the BitSet pointer.
	// TODO: check the validation.
	BitSet* bitSet = (BitSet*)data;

	// Read from the file.
	fin.read((char*)&bitSet->mArraySize, sizeof(int));
	fin.read((char*)&bitSet->mArray[0], sizeof(int) * bitSet->mArraySize);
	fin.read((char*)&bitSet->mBitCount, sizeof(int));

	return true;
}