#include "Common.h"

#include "Math.h"
#include "Utility.h"
#include "Algorithm.h"
#include "GridAccelerated.h"
#include "BentleyOttmannAccelerated.h"
#include "BitSet.h"
#include <ctime>
#include <iostream>
#include <iomanip>
#include <map>
#include <string>
#include <fstream>

#define _OUTPUT_TO_FILE_ 1
#define _SAVE_POLYGON_SEGMENT_ 1
#define _LOAD_POLYGON_SEGMENT_ 0

#define _USE_GRID_ 1

// Name of different LineSegment-(Convex)Polygon-Intersection algorithms.
enum EAlgorithm
{
    BRUTE_FORCE             = 0,
    OPTIMIZED_BRUTE_FORCE,
    SIMPLE_SAT,
    OPTIMIZED_SAT,
    EXTREME_SAT,
    GJK_BASED_SAT,
    GJK_BASED_SAT_EX,
    GJK_BASED_RAYCAST,
    BINARY_SEARCH_BY_EXTREMES,
    BINARY_SEARCH,
    EXTREME_PLUS_POINT_INSIDE,
    LIFO,
#if _USE_GRID_
    GRID_ACCELERATED,
#endif
    BENTLEY_OTTMANN_ACCELERATED,

    TOTAL_ALGORITHM_COUNT,
};

std::string gAlgorithmNames[] = 
{
    "BruteForce",
    "OptimizedBruteForce",
    "SimpleSAT",
    "OptimizedSAT",
    "ExtremeSAT",
    "GJKBasedSAT",
    "GJKBasedSATEx",
    "GJKBasedRayCast",
    "BinarySearchByExtremes",
    "BinarySearch",
    "ExtremePlusPointInside",
    "LIFO",
#if _USE_GRID_
    "GridAccelerated",
#endif
	"BentleyOttmannAccelerated"
};
int g_nameMaxLength = 32;

// Vertices count.
int g_count = 100;
// Hit ratio.
float g_ratio = 0.5f;
// Polygon number.
int g_polygonNumber = 25;
// Line segment number.
int g_segmentNumber = 100;
// Radius of the regular polygon.
float g_radius = 1.0f;

// Tolerance.
float g_tolerance = 0.02f;


/////////////////////////////////////////////////
// Main test.
/////////////////////////////////////////////////
int main(int argc, char* argv[])
{   
    LARGE_INTEGER performanceCount;
    QueryPerformanceFrequency(&performanceCount);
    double freqency = (double)(performanceCount.QuadPart);
    srand((unsigned)time(NULL));
    
    
    // Allocate memory for polygons and line segments.
    Vector2** polygons = new Vector2*[g_polygonNumber];
    for (int i = 0; i < g_polygonNumber; i++)
    {
#if _ACT_AS_UNCLOSED_
        polygons[i] = new Vector2[g_count + 1];
#else
        polygons[i] = new Vector2[g_count];
#endif
    }

    Vector2** segmentsList = new Vector2*[g_polygonNumber];
    for (int i = 0; i < g_polygonNumber; i++)
    {
        segmentsList[i] = new Vector2[2 * g_segmentNumber];
    }

	BitSet* hitFlagsList = new BitSet[g_polygonNumber];
	for (int i = 0; i < g_polygonNumber; ++i)
	{
		hitFlagsList[i].Initialize(unsigned int(g_segmentNumber));
	}

#if _LOAD_POLYGON_SEGMENT_
	// Load polygons and line segments.
	LoadPolygonSegments(g_polygonNumber, g_count, polygons,
		g_segmentNumber, segmentsList, hitFlagsList, "data.dat");
#else
    // Generate polygons and line segments.
    GeneratePolygonSegments(g_polygonNumber, g_count, g_radius, polygons, 
        g_segmentNumber, g_ratio, segmentsList, hitFlagsList);
#endif

#if _ACT_AS_UNCLOSED_
    for (int i = 0; i < g_polygonNumber; i++)
        polygons[i][g_count] = polygons[i][0];
#endif
    
#if _SAVE_POLYGON_SEGMENT_
	// Binary.
	SavePolygonSegments(g_polygonNumber, g_count, polygons, g_segmentNumber, segmentsList, hitFlagsList,
		"data.dat", true);
	// Ascii.
	SavePolygonSegments(g_polygonNumber, g_count, polygons, g_segmentNumber, segmentsList, hitFlagsList,
		"data.txt", false);
#endif

    // Allocate memory for store the time consumed.
    long long* consumeTime = new long long[TOTAL_ALGORITHM_COUNT];
    for (int i = 0; i < TOTAL_ALGORITHM_COUNT; i++)
    {
        consumeTime[i] = 0;
    }

    // For each algorithm, store the errors of each polygon. 
    // This is used to verify the correctness of the algorithm.
	BitSet** errorFlags = new BitSet*[TOTAL_ALGORITHM_COUNT];
	for (int i = 0; i < TOTAL_ALGORITHM_COUNT; ++i)
	{
		errorFlags[i] = new BitSet[g_polygonNumber];
		for (int j = 0; j < g_polygonNumber; j++)
		{
			errorFlags[i][j].Initialize(g_segmentNumber);
		}
	}
	// Error ratio.
	float* errorRatio = new float[TOTAL_ALGORITHM_COUNT];
	for (int i = 0; i < TOTAL_ALGORITHM_COUNT; ++i)
		errorRatio[i] = 0.0f;
    
    // Create algorithm list and run test program.
    SegmentPolygonTest* algorithmList = new SegmentPolygonTest[TOTAL_ALGORITHM_COUNT];
    // Brute force.
    algorithmList[BRUTE_FORCE] = SegmentPolygonIntersection_BruteForce;
    algorithmList[OPTIMIZED_BRUTE_FORCE] = SegmentPolygonIntersection_OptimizedBruteForce;
    // SAT.
    algorithmList[SIMPLE_SAT]                   = SegmentPolygonIntersection_SimpleSAT;
    algorithmList[OPTIMIZED_SAT]                = SegmentPolygonIntersection_OptimizedSAT;    
    algorithmList[EXTREME_SAT]                  = SegmentPolygonIntersection_ExtremeSAT;
    algorithmList[GJK_BASED_SAT]                = SegmentPolygonIntersection_GJKBasedSAT;
    algorithmList[GJK_BASED_SAT_EX]             = SegmentPolygonIntersection_GJKBasedSATEx;
    algorithmList[GJK_BASED_RAYCAST]		    = SegmentPolygonIntersection_GJKBasedRayCast;
    // Binary Search.
    algorithmList[BINARY_SEARCH_BY_EXTREMES]    = SegmentPolygonIntersection_BinarySearchByExtremes;
    algorithmList[BINARY_SEARCH]                = SegmentPolygonIntersection_BinarySearch;
    // Others.
    algorithmList[EXTREME_PLUS_POINT_INSIDE]    = SegmentPolygonIntersection_ExtremePlusPointInside;
    algorithmList[LIFO]                         = SegmentPolygonIntersection_LIFO;
#if _USE_GRID_
    algorithmList[GRID_ACCELERATED]             = SegmentPolygonIntersection_GridAccelerated;
#endif
	algorithmList[BENTLEY_OTTMANN_ACCELERATED]  = SegmentPolygonIntersection_BentleyOttmannAccelerated;

    
    RunSegmentPolygonAlgorithms(g_polygonNumber, g_count, polygons, 
        g_segmentNumber, segmentsList,
        algorithmList, TOTAL_ALGORITHM_COUNT, consumeTime, 
		hitFlagsList, errorFlags);

    
    // Print result.
    // Firstly, check whether the test results of all algorithms are right.
    // For each algorithm, compare the hit counts of each polygon to exected figure (ratio * segmentNumber), 
    // if not equal, print the difference.
#if _OUTPUT_TO_FILE_
    std::ofstream fout("result.txt");
#endif
    bool error = false;
    for (int i = 0; i < TOTAL_ALGORITHM_COUNT; i++)
    {
        for (int j = 0; j < g_polygonNumber; j++)
        {
			int diff = errorFlags[i][j].Number();
			if (diff > 0)
			{
				if (diff > g_segmentNumber * g_tolerance)
				{
					error = true;
					std::cout << "Error in algorithm" << i << " - " << gAlgorithmNames[i] << ": " << std::endl
						<< "Difference in hit g_count of polygon[" << j << "]: " << diff << std::endl;
#if _OUTPUT_TO_FILE_
					fout << "Error in algorithm" << i << " - " << gAlgorithmNames[i] << ": " << std::endl
						<< "Difference in hit g_count of polygon[" << j << "]: " << diff << std::endl;
#endif
				}
				else
				{
					// Sum the error number for each polygon.
					errorRatio[i] += float(diff);
				}
			}			
        }

		// Calculate the final error ratio.
		errorRatio[i] /= g_polygonNumber * g_segmentNumber;
    }
    // If no error, print the time used by each algorithm (from fastest to slowest).
    if (!error)
    {
		std::cout << "Regular polygon vertex count: " << g_count << "; " << std::endl
            << "Regular polygon radius: " << g_radius << "; " << std::endl
            << "Hit ratio: " << g_ratio << "; " << std::endl
            << "Polygon number: " << g_polygonNumber << "; " << std::endl
            << "Line segment number: " << g_segmentNumber << ". " << std::endl << std::endl;
#if _OUTPUT_TO_FILE_
		fout << "Regular polygon vertex count: " << g_count << "; " << std::endl
            << "Regular polygon radius: " << g_radius << "; " << std::endl
            << "Hit ratio: " << g_ratio << "; " << std::endl
            << "Polygon number: " << g_polygonNumber << "; " << std::endl
            << "Line segment number: " << g_segmentNumber << ". " << std::endl << std::endl;
#endif
	
        std::map<double, int, std::less<double>> orderedAlgorithmList;
        for (int i = 0; i < TOTAL_ALGORITHM_COUNT; i++)
        {
            orderedAlgorithmList.insert(std::pair<double, int>((double)consumeTime[i] / freqency, i));
        }
        std::map<double, int, std::less<double>>::iterator itor;
        for (itor = orderedAlgorithmList.begin(); itor != orderedAlgorithmList.end(); itor++)
        {
            std::cout << std::setiosflags(std::ios::left) 
				      << std::setw(g_nameMaxLength) << gAlgorithmNames[itor->second] << ": " 
					  << std::setw(10) << itor->first << "s; " 
					  << errorRatio[itor->second] << "." << std::endl;
#if _OUTPUT_TO_FILE_
            fout << std::setiosflags(std::ios::left) 
				 << std::setw(g_nameMaxLength) << gAlgorithmNames[itor->second] << ": " 
				 << std::setw(10) << itor->first << "s." 
				 << errorRatio[itor->second] << "." << std::endl;
#endif
        }
        std::cout << std::endl;
    }
#if _OUTPUT_TO_FILE_
    fout.close();
#endif


    // Release resources.
    for (int i = 0; i < g_polygonNumber; i++)
    {
        delete [] polygons[i];    
    }
    delete [] polygons;
    for (int i = 0; i < g_polygonNumber; i++)
    {        
        delete [] segmentsList[i];
    }
    delete [] segmentsList;
	delete [] hitFlagsList;

    delete [] consumeTime;
	
    for (int i = 0; i < TOTAL_ALGORITHM_COUNT; i++)
    {
        delete [] errorFlags[i];
    }
    delete [] errorFlags;
    delete [] algorithmList;

    return 0;
}