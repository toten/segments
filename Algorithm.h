#ifndef _ALGORITHM_H_
#define _ALGORITHM_H_

#include "Math.h"

#define _PRE_MINMAX_ 1
#define _ACT_AS_UNCLOSED_ 1

#define MAX_ITERATIONS 32




/*----------------------------------------------------------------------------------------------------
 * Basic algorithms for verification.
 *--------------------------------------------------------------------------------------------------*/

/// <description>
/// Enumeration of basic segment-polygon result.
/// </description>
enum EResult
{
    /// <description>
    /// No intersection.
    /// </description>
    Miss            = 0,

    /// <description>
    /// At least one vertex inside.
    /// </description>
    PointInside    = 1,

    /// <description>
    /// No vertex inside, but edge intersects.
    /// </description>
    EdgeIntersect   = 2
};

/// <description>
/// Basic Segment-Polygon-Intersection algorithm. 
/// This method is only for generate test cases but not for comparison.
/// This is a purely brute force approach which loops edges of the polygon
/// and test whether any one of them intersects the line segment. 
/// So it is totally based on segment-segment intersection algorithm.
/// </description>
EResult SegmentPolygonIntersection_Basic(Vector2 p0, Vector2 p1, const Vector2* polygon, int count);    





/*----------------------------------------------------------------------------------------------------
 * Segment-Polygon-Intersection algorithms
 *--------------------------------------------------------------------------------------------------*/

/// <description>
/// Define function pointer for the following algorithm which will go into comparison.
/// </description>
typedef bool (*SegmentPolygonTest)(Vector2 p0, Vector2 p1, const Vector2* polygon, int count);

/////////////////////////////////////////////////////
// Brute force approaches.
/////////////////////////////////////////////////////

/// <description>
/// This algorithm is same to the basic one.
/// First, check whether any one endpoint of the line segment is inside the polygon.
/// If none, test whether the line segment intersects any edge of the polygon.
/// </description>
bool SegmentPolygonIntersection_BruteForce(Vector2 p0, Vector2 p1, const Vector2* polygon, int count);

/// <description>
/// This algorithm is similar to the previous one, except early rejection by testing separation 
/// along x or y axis when do segment-segment test.
/// </description>
bool SegmentPolygonIntersection_OptimizedBruteForce(Vector2 p0, Vector2 p1, const Vector2* polygon, int count);



/////////////////////////////////////////////////////
// Separating-Axis-Test approaches.
/////////////////////////////////////////////////////

/// <description>
/// Simple Separating-Axis-Test, and no optimization.
/// </description>
bool SegmentPolygonIntersection_SimpleSAT(Vector2 p0, Vector2 p1, const Vector2* polygon, int count);

/// <description>
/// Optimized Separating-Axis-Test.
/// </description>
bool SegmentPolygonIntersection_OptimizedSAT(Vector2 p0, Vector2 p1, const Vector2* polygon, int count);

/// <description>
/// Optimize SAT by finding extreme point.
/// </description>
bool SegmentPolygonIntersection_ExtremeSAT(Vector2 p0, Vector2 p1, const Vector2* polygon, int count);

/// <description>
/// SAT based on GJK algorithm.
/// </description>
bool SegmentPolygonIntersection_GJKBasedSAT(Vector2 p0, Vector2 p1, const Vector2* polygon, int count);

/// <description>
/// SAT based on GJK algorithm improved.
/// </description>
bool SegmentPolygonIntersection_GJKBasedSATEx(Vector2 p0, Vector2 p1, const Vector2* polygon, int count);

/// <description>
/// GJK based raycast.
/// </description>
bool SegmentPolygonIntersection_GJKBasedRayCast(Vector2 p0, Vector2 p1, const Vector2* polygon, int count);


/////////////////////////////////////////////////////
// Binary-Search approaches.
/////////////////////////////////////////////////////

/// <description>
/// First, divide polygon into two chain collected by minimum and maximum extreme points along normal vector
/// of the given segment. Then, do binary search for each chain to find the exact one edge that probably intersects
/// the line segment.
/// </description>
bool SegmentPolygonIntersection_BinarySearchByExtremes(Vector2 p0, Vector2 p1, const Vector2* polygon, int count);

/// <description>
/// This algorithm use binary search to find the exact 2 edges of the polygon which can have intersections 
/// with the given line segment. Then, do 2 times segment-segment-intersection test.
/// </description>
bool SegmentPolygonIntersection_BinarySearch(Vector2 p0, Vector2 p1, const Vector2* polygon, int count);




/////////////////////////////////////////////////////
// Binary-Search approaches.
/////////////////////////////////////////////////////

/// <description>
/// Line segment to convex polygon test by doing point-in-polygon first.
/// </description>
bool SegmentPolygonIntersection_ExtremePlusPointInside(Vector2 p0, Vector2 p1, const Vector2* polygon, int count);

/// <description>
/// LIFO(Last-In-First-Out) algorithm.
/// </description>
bool SegmentPolygonIntersection_LIFO(Vector2 p0, Vector2 p1, const Vector2* polygon, int count);




/////////////////////////////////////////////////////
// Untility methods.
/////////////////////////////////////////////////////

/// <description>
/// Swap.
/// </description>
template <class T>
inline void Swap(T& p, T& q)
{
    T tmp = p;
    p = q;
    q = tmp;
}

/// <description>
/// Antonio algorithm to solve 2 line segments intersection.
/// </description>
bool SegmentSegmentIntersection_Antonio_Direction(Vector2 start0, Vector2 d0, Vector2 start1, Vector2 d1);

/// <description>
/// Antonio algorithm to solve 2 line segments intersection.
/// </description>
bool SegmentSegmentIntersection_Antonio(Vector2 start0, Vector2 end0, Vector2 start1, Vector2 end1);

/// <description>
/// Based on Antonio algirthm, but do early rejection by checking separating along x/y axis.
/// </description>
bool SegmentSegmentIntersection_EarlyReject(Vector2 start0, Vector2 end0, Vector2 start1, Vector2 end1);

/// <description>
/// Basic crossings test to test point-in-polygon.
/// </description>
bool PointInPolygon_Crossings(Vector2 point, const Vector2* polygon, int count);

/// <description>
/// Binary search
/// </description>
void ChainBinarySearch(const Vector2* polygon, int count, int front, int end, Vector2 dir, Vector2 pivot, float sign, int& p, int& q);

/// <description>
/// Project polygon to given vector and compute interval.
/// </description>
void ComputeInterval(const Vector2* polygon, int count, Vector2 dir, Vector2 pivot, float& min, float& max);

/// <description>
/// Project polygon to given vector and see whether the interval is completely positive.
/// </description>
int ComputeInterval_Optimized(const Vector2* polygon, int count, Vector2 dir, Vector2 pivot);

/// <description>
/// Find maximum extreme point of the polygon according to given direction.
/// </description>
int FindMaximum(const Vector2* polygon, int count, Vector2 dir);

/// <description>
/// Crossings test for convex polygon. 
/// Stop when find the 2 possible crossings.
/// </description>
bool PointInPolygon_CrossingsConvex(Vector2 point, const Vector2* polygon, int count);

/// <description>
/// Clip line segment by the box.
/// </description>
bool ClipSegment(const Vector2& p0, const Vector2& p1, 
				 const Vector2& minCorner, const Vector2& maxCorner, 
				 Vector2& pa, Vector2& pb);

/// <description>
/// Line-Slab intersection method.
/// </description>
bool LineSlabIntersect(float start, float direction, float minSide, float maxSide, float& tMin, float& tMax);

/// <description>
/// Find the bounding box of the given line segment.
/// </description>
void BoundingBoxOfSegment(const Vector2& p0, const Vector2& p1, const Vector2& dir, Vector2& minCorner, Vector2& maxCorner);

/// <description>
/// Box-box intersection test.
/// </description>
bool BoxBoxIntersect(const Vector2& min0, const Vector2& max0, const Vector2& min1, const Vector2& max1);

/// <description>
/// Find the support point on minkowski difference between the polygon and the line segment in -direction.
/// </description>
Vector2 Support(const Vector2& p0, const Vector2& p1, const Vector2* polygon, int count, const Vector2& direction);

/// <description>
/// Find the support point on minkowski difference between the polygon and the point in -direction.
/// </description>
Vector2 Support(const Vector2& point, const Vector2* polygon, int count, const Vector2& direction);

#endif // Once.