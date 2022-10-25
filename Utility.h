#ifndef _UTILITY_H_
#define _UTILITY_H_

#include "Math.h"
#include "Algorithm.h"
#include "windows.h"
#include "BitSet.h"

/// <description>
/// Generate regular polygon.
/// </description>
void GenerateRegularPolygon(int count, float offset, float radius, Vector2* polygon, Vector2& min, Vector2& max);

/// <description>
/// Generate test segments of given quantity and meets the intersection ratio when comparing to the polygon.
/// </description>
void GenerateSegments(const Vector2* polygon, int count, 
                      const Vector2& min, const Vector2& max, 
                      int segmentNumber, float ratio, Vector2* segments, BitSet& hitFlags);

/// <description>
/// Generate polygons-segmentsList for test.
/// </description>
void GeneratePolygonSegments(int polygonNumber, int count, float radius, Vector2** polygons, 
                             int segmentNumber, float ratio, Vector2** segmentsList, BitSet* hitFlagsList);

/// <description>
/// For given polygons-segmentsList, run different algorithms and get time consumed for each other.
/// </description>
void RunSegmentPolygonAlgorithms(int polygonNumber, int count, Vector2** polygons,
                                 int segmentNumber, Vector2** segmentsList,
                                 SegmentPolygonTest* algorithmList, int algorithmCount, long long* consumeTime, 
								 BitSet* hitFlagsList, BitSet** errorFlags);

/// <description>
/// Save the polygon and the segmentsList to file.
/// TODO: let the 2d arrays const.
/// TODO: how to do "reverse".
/// </description>
void SavePolygonSegments(int polygonNumber, int count, Vector2** polygons,
						 int segmentNumber, Vector2** segmentsList, 
						 const BitSet* hitFlagsList,
						 const char* datafile, bool isBinary = true,
						 int* indices = NULL, int indexCount = 0/*, bool isReverse = false*/);

/// <description>
/// Load the polygon and the segmentsList from file.
/// </description>
bool LoadPolygonSegments(int polygonNumber, int count, Vector2** polygons,
						 int segmentNumber, Vector2** segmentList,
						 BitSet* hitFlagsList,
						 const char* datafile, 
						 int* indices = NULL, int indexCount = 0);

#endif // Once.