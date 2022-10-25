#include "Common.h"

#include "Utility.h"
#include <fstream>
#include <iomanip>

const float EXPAND_RATIO = 1.2f;

/// <description>
/// Generate regular polygon.
/// </description>
void GenerateRegularPolygon(int count, float offset, float radius, Vector2* polygon, Vector2& min, Vector2& max)
{
    min.x = INFINITE_VALUE;
    min.y = INFINITE_VALUE;
    max.x = -INFINITE_VALUE;
    max.y = -INFINITE_VALUE;

    float unitAngle = 2.0f * PI /  count;

    for (int i = 0; i < count; i++)
    {
        float angle = i * unitAngle + offset;
        polygon[i].x = cos(angle) * radius;
        polygon[i].y = sin(angle) * radius;

        // Get min/max.
        if (polygon[i].x < min.x)
            min.x = polygon[i].x;
        if (polygon[i].x > max.x)
            max.x = polygon[i].x;
        if (polygon[i].y < min.y)
            min.y = polygon[i].y;
        if (polygon[i].y > max.y)
            max.y = polygon[i].y;
    }
}

/// <description>
/// Generate test segments of given quantity and meets the intersection ratio when comparing to the polygon.
/// </description>
void GenerateSegments(const Vector2* polygon, int count, 
                      const Vector2& min, const Vector2& max, 
                      int segmentNumber, float ratio, Vector2* segments, BitSet& hitFlags)
{
    // Get the hit/miss number.
    int hitNumber = (int)(ratio * (float)segmentNumber);
    int missNumber = segmentNumber - hitNumber;

    int hitCount = 0;
    int missCount = 0;
    for (int i = 0; i < segmentNumber;)
    {
        // Randomly get the 2 endpoints of the segment.
        Vector2 p0, p1;
        p0.x = Random() * (max.x - min.x) + min.x;
        p0.y = Random() * (max.y - min.y) + min.y;
        p1.x = Random() * (max.x - min.x) + min.x;
        p1.y = Random() * (max.y - min.y) + min.y;

        bool isHit = false;
        // Do basic test.
        EResult result = SegmentPolygonIntersection_Basic(p0, p1, polygon, count);
        if (result == EdgeIntersect)
        {
            isHit = true;
        }
        else if (result == PointInside)
        {
#ifdef _ENDPOINTS_OUTSIDE_
            continue;
#else
            isHit = true;
#endif
        }
        else
        {
            isHit = false;
        }

        // Update statistices.
        if ((isHit && (hitCount < hitNumber)) ||
            (!isHit && (missCount < missNumber)))
        {
            segments[2 * i] = p0;
            segments[2 * i + 1] = p1;

            if (isHit)
			{
                hitCount++;

				// Set flag.
				hitFlags.SetBit(i);
			}
            else
			{
                missCount++;
			}

            i++;
        }
    }
}

/// <description>
/// Generate polygons-segmentsList for test.
/// </description>
void GeneratePolygonSegments(int polygonNumber, int count, float radius, Vector2** polygons, 
                             int segmentNumber, float ratio, Vector2** segmentsList, BitSet* hitFlagsList)
{
    for (int i = 0; i < polygonNumber; i++)
    {
        // Create regular polygon with random offset.
        float offset = Random() * 2.0f * PI;
        Vector2 min, max;
        GenerateRegularPolygon(count, offset, radius, polygons[i], min, max);

        // create segments for test.
        min *= EXPAND_RATIO;
        max *= EXPAND_RATIO;
        GenerateSegments(polygons[i], count, min, max, segmentNumber, ratio, segmentsList[i], hitFlagsList[i]);
    }
}

/// <description>
/// For given polygons-segmentsList, run different algorithms and get time consumed for each other.
/// </description>
void RunSegmentPolygonAlgorithms(int polygonNumber, int count, Vector2** polygons,
                                 int segmentNumber, Vector2** segmentsList,
                                 SegmentPolygonTest* algorithmList, int algorithmCount, long long* consumeTime,
								 BitSet* hitFlagsList, BitSet** errorFlags)
{    
    for (int i = 0; i < polygonNumber; i++)
    {
        // Get polygon and segments.
        const Vector2* polygon = polygons[i];
        const Vector2* segments = segmentsList[i];
		
        for (int j = 0; j < segmentNumber; j++)
        {
            // Get segment (start, end).
            Vector2 start = segments[2 * j];
            Vector2 end = segments[2 * j + 1];

			// Get the base hit flag.
			bool base = hitFlagsList[i].CheckBit(j);
            for (int k = 0; k < algorithmCount; k++)
            {
                // Run algorithm.
                // Record start time.
                LARGE_INTEGER performanceCount;
                QueryPerformanceCounter(&performanceCount);
                long long startCount = performanceCount.QuadPart;
                
                bool hit = (*algorithmList[k])(start, end, polygon, count);
                
                // Record end time.
                QueryPerformanceCounter(&performanceCount);
                long long endCount = performanceCount.QuadPart;
                consumeTime[k] += endCount - startCount;
                
                // Record error.				
                if (hit != base)
                {
					errorFlags[k][i].SetBit(j);
                }
            }
        }
    }
}

/// <description>
/// Save the polygon and the segmentsList to file.
/// </description>
void SavePolygonSegments(int polygonNumber, int count, Vector2** polygons,
						 int segmentNumber, Vector2** segmentsList,
						 const BitSet* hitFlagsList,
						 const char* datafile, bool isBinary,
						 int* indices, int indexCount/*, bool isReverse*/)
{
	// If indices is NULL, save all the polygon-segments.
	int* _indices = indices;
	if (_indices == NULL)
	{
		_indices = new int[polygonNumber];
		for (int i = 0; i < polygonNumber; ++i)
			_indices[i] = i;

		indexCount = polygonNumber;
	}

	// Create file stream.
	std::ofstream fout;
	std::ios_base::openmode mode = std::ios_base::out;
	if (isBinary)
		mode |= std::ios_base::binary;
	fout.open(datafile, mode);

	// Write the title.
	if (isBinary)
	{
		fout.write((const char*)&polygonNumber, sizeof(polygonNumber));
		fout.write((const char*)&indexCount, sizeof(indexCount));
		fout.write((const char*)&count, sizeof(count));
		fout.write((const char*)&segmentNumber, sizeof(segmentNumber));		
	}
	else
	{
		fout << "polygon total number:" << polygonNumber << std::endl;
		fout << "polygon number: " << indexCount << std::endl;
		fout << "vertex count: " << count << std::endl;
		fout << "segment number: " << segmentNumber << std::endl;
	}

	// Loop the polygon.
	BitSetSerialization bitSetSerialization;
	for (int i = 0; i < indexCount; ++i)
	{
		int index = _indices[i];
		// TODO: Assert(0 <= index < polygonNumber);
		
		// Get the polygon, segments and hitFlags.
		const Vector2* polygon = polygons[index];
		const Vector2* segments = segmentsList[index];
		const BitSet& hitFlags = hitFlagsList[index];
		if (isBinary)
		{
			// Write to binary file.
			fout.write((const char*)&index, sizeof(index));
			fout.write((const char*)&polygon[0], sizeof(Vector2) * count);
			fout.write((const char*)&segments[0], sizeof(Vector2) * segmentNumber * 2);

			// Serialize the hitFlags.			
			bitSetSerialization.Serialize(&hitFlags, fout);
		}
		else
		{
			// Write to ascii file.			
			fout << "================================================" << std::endl;
			fout << "polygon[" << index << "]:" << std::endl;
			for (int j = 0; j < count; ++j)
			{
				fout << "(" << std::setiosflags(std::ios::left)
					<< std::setw(6) << polygon[j].x << ","
					<< std::setw(6) << polygon[j].y << "), ";
				
				if ((j + 1) % 4 == 0)
					fout << std::endl;
			}
			fout << std::endl;
			fout << "segments:" << std::endl;
			for (int j = 0; j < segmentNumber; ++j)
			{
				// Start point.
				fout << "(" << std::setiosflags(std::ios::left)
					<< std::setw(6) << segments[j * 2].x << ","
					<< std::setw(6) << segments[j * 2].y << "), ";					

				// End point.
				fout << "(" << std::setiosflags(std::ios::left)
					<< std::setw(6) << segments[j * 2 + 1].x << ","
					<< std::setw(6) << segments[j * 2 + 1].y << "), ";					
				
				fout << (hitFlags.CheckBit(j) ? "1" : "0") << std::endl;
			}
			fout << std::endl << std::endl;
		}
	}

	fout.close();

	// Release.
	if (indices == NULL)
		delete [] _indices;
}

/// <description>
/// Load the polygon and the segmentsList from file.
/// </description>
bool LoadPolygonSegments(int polygonNumber, int count, Vector2** polygons,
						 int segmentNumber, Vector2** segmentList,
						 BitSet* hitFlagsList,
						 const char* datafile, 
						 int* indices, int indexCount)
{
	int* _indices = indices;
	if (_indices == NULL)
	{
		_indices = new int[polygonNumber];
		for (int i = 0; i < polygonNumber; ++i)
			_indices[i] = i;
		indexCount = polygonNumber;
	}

	// Create file stream.
	std::ifstream fin;
	fin.open(datafile, std::ios_base::in | std::ios_base::binary);

	// Read the title.
	int pn, ic, c, sn;
	fin.read((char*)&pn, sizeof(pn));
	fin.read((char*)&ic, sizeof(ic));
	fin.read((char*)&c, sizeof(c));
	fin.read((char*)&sn, sizeof(sn));
	// TODO: Assert(pn == polygonNumber && ic == indexCount && c == count && sn == segmentNumber);

	// Loop the polygon.
	BitSetSerialization bitSetSerialization;
	for (int i = 0; i < indexCount; ++i)
	{
		int index = _indices[i];
		// TODO: Assert(0 <= index < polygonNumber);

		// Get the polygon, segments and hitFlags.
		Vector2* polygon = polygons[index];
		Vector2* segments = segmentList[index];
		BitSet& hitFlags = hitFlagsList[index];

		// Read from the file.
		int idx;
		fin.read((char*)&idx, sizeof(idx));
		// TODO: Assert(idx == index);
		fin.read((char*)&polygon[0], sizeof(Vector2) * count);
		fin.read((char*)&segments[0], sizeof(Vector2) * segmentNumber * 2);
		
		// Deserialize the hitFlags.
		bitSetSerialization.Deserialize(&hitFlags, fin);
	}

	fin.close();

	// Release.
	if (indices == NULL)
		delete [] _indices;
	
	return true;
}