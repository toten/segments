#include "Common.h"

#include "Algorithm.h"
#include "GJK.h"
#include "VoronoiSimplexSolver.h"

#define GJK_PROFILE 1
#if GJK_PROFILE
int counter_loop_GJK = 0;
int counter_loop_GJKEx = 0;
int counter_loop_RayCast = 0;
#endif

/// <description>
/// Basic Segment-Polygon-Intersection algorithm.
/// This is a purely brute force approach which loops edges of the polygon
/// and test whether any one of them intersects the line segment. 
/// So it is totally based on segment-segment intersection algorithm.
/// </description>
EResult SegmentPolygonIntersection_Basic(Vector2 p0, Vector2 p1, const Vector2* polygon, int count)
{
    // Test point inside first.
    if (PointInPolygon_Crossings(p0, polygon, count) ||
        PointInPolygon_Crossings(p1, polygon, count))
    {
        return PointInside;
    }

    // No points inside, test edge intersect.
    Vector2 v0 = polygon[count - 1];
    Vector2 v1(0.0f, 0.0f);
    for (int i = 0; i < count; i++)
    {
        v1 = polygon[i];

        // Test (start, end) & (p0, p1) intersect or not.
        if (SegmentSegmentIntersection_Antonio(v0, v1, p0, p1))
        {
            return EdgeIntersect;
        }

        v0 = v1;
    }
    
    return Miss;
}

/// <description>
/// This algorithm is same to the basic one.
/// First, check whether any one endpoint of the line segment is inside the polygon.
/// If none, test whether the line segment intersects any edge of the polygon.
/// </description>
bool SegmentPolygonIntersection_BruteForce(Vector2 p0, Vector2 p1, const Vector2* polygon, int count)
{
#ifndef _ENDPOINTS_OUTSIDE_
    // Test point inside first.
    if (PointInPolygon_Crossings(p0, polygon, count) ||
        PointInPolygon_Crossings(p1, polygon, count))
    {
        return true;
    }
#endif

    // No points inside, test edge intersect.
    Vector2 v0 = polygon[count - 1];
    Vector2 v1(0.0f, 0.0f);
    for (int i = 0; i < count; i++)
    {
        v1 = polygon[i];

        // Test (start, end) & (p0, p1) intersect or not.
        if (SegmentSegmentIntersection_Antonio(v0, v1, p0, p1))
        {
            return true;
        }

        v0 = v1;
    }
    
    return false;
}

/// <description>
/// This algorithm is similar to the previous one, except early rejection when do segment-segment test.
/// 2 slightly different approaches are shown below. 
/// (1) If _PRE_MINMAX_ defined, compute min/max of given line segment(p0, p1) outside the polygon edge's loop.
/// (2) else, defer the min/max computation to method SegmentSegmentIntersection_EarlyReject, but the min/max
///     for (p0, p1) is repeatedly calculated a lot of times.
/// </description>
bool SegmentPolygonIntersection_OptimizedBruteForce(Vector2 p0, Vector2 p1, const Vector2* polygon, int count)
{
#ifndef _ENDPOINTS_OUTSIDE_
    // Test point inside first.
    if (PointInPolygon_Crossings(p0, polygon, count) ||
        PointInPolygon_Crossings(p1, polygon, count))
    {
        return true;
    }
#endif

#if _PRE_MINMAX_
    // Pre-compute: the min/max of (p0, p1).
    // It is better to do it once outside the polygon edge loop.
    Vector2 min = p0;
    Vector2 max = p1;
    MinMax(min.x, max.x);
    MinMax(min.y, max.y);
#endif

    // No points inside, test edge intersect.
    Vector2 v0 = polygon[count - 1];
    Vector2 v1(0.0f, 0.0f);
    for (int i = 0; i < count; i++)
    {
        v1 = polygon[i];

#if _PRE_MINMAX_
        // Test separation along x and y axis for early rejection.
        float minValue = v0.x;
        float maxValue = v1.x;
        MinMax(minValue, maxValue);
        if (minValue > max.x || maxValue < min.x)
        {
            v0 = v1;
            continue;
        }
        minValue = v0.y;
        maxValue = v1.y;
        MinMax(minValue, maxValue);
        if (minValue > max.y || maxValue < min.y)
        {
            v0 = v1;
            continue;
        }
#endif

        // Test (start, end) & (p0, p1) intersect or not.
#if _PRE_MINMAX_
        if (SegmentSegmentIntersection_Antonio(v0, v1, p0, p1))
#else
        if (SegmentSegmentIntersection_EarlyReject(v0, v1, p0, p1))
#endif
        {
            return true;
        }

        v0 = v1;
    }
    
    return false;
}

/// <description>
/// Simple Separating-Axis-Test, and no optimization.
/// </description>
bool SegmentPolygonIntersection_SimpleSAT(Vector2 p0, Vector2 p1, const Vector2* polygon, int count)
{
    // Test separating along normal vector of p1 - p0.
    Vector2 dir = p1 - p0;
    Vector2 normal(dir.y, -dir.x);
    float min = 0.0f;
    float max = 0.0f;
    ComputeInterval(polygon, count, normal, p0, min, max);
    if (min > 0.0f || max < 0.0f)
    {
        return false;
    }

    // Test separating along normal vector of polygon[i] - polygon[i-1].
    Vector2 v0 = polygon[count - 1];
    Vector2 v1;
    for (int i = 0; i < count; i++)
    {
        v1 = polygon[i];
        dir = v1 - v0;
        normal.x = dir.y;
        normal.y = -dir.x;

        // Project line segment to get interval.
        float segMin = Vec2Dot(p0 - v0, normal);
        float segMax = Vec2Dot(p1 - v0, normal);
        MinMax(segMin, segMax);

        // Project polygon to get interval.
        ComputeInterval(polygon, count, normal, v0, min, max);

        // Compare.
        if (segMin > max || min > segMax)
        {
            return false;
        }

        v0 = v1;
    }    

    return true;
}

/// <description>
/// Optimized Separating-Axis-Test.
/// </description>
bool SegmentPolygonIntersection_OptimizedSAT(Vector2 p0, Vector2 p1, const Vector2* polygon, int count)
{
    // Test separating along normal vector of p1 - p0.
    Vector2 dir = p1 - p0;
    Vector2 normal(dir.y, -dir.x);
    int side = ComputeInterval_Optimized(polygon, count, normal, p0);
    if (side == 1 || side == -1)
    {
        return false;
    }

    // Test separating along normal vector of polygon[i] - polygon[i-1].
    Vector2 v0 = polygon[count - 1];
    Vector2 v1;
    for (int i = 0; i < count; i++)
    {
        v1 = polygon[i];
        dir = v1 - v0;
        normal.x = dir.y;
        normal.y = -dir.x;

        // Project line segment to get interval.
        float value = Vec2Dot(p0 - v0, normal);
        if (value <= 0.0f)
        {
            v0 = v1;
            continue;
        }
        value = Vec2Dot(p1 - v0, normal);
        if (value <= 0.0f)
        {
            v0 = v1;
            continue;
        }
        return false;
    }    

    return true;
}

/// <description>
/// Optimize SAT by finding extreme point.
/// </description>
bool SegmentPolygonIntersection_ExtremeSAT(Vector2 p0, Vector2 p1, const Vector2* polygon, int count)
{
    // Test separating along normal vector of p1 - p0.
    Vector2 dir = p1 - p0;
    Vector2 normal(dir.y, -dir.x);
    int maxExtreme = FindMaximum(polygon, count, normal);
    if (Vec2Dot(normal, polygon[maxExtreme] - p0) < 0.0f)
        return false;
    int minExtreme = FindMaximum(polygon, count, -normal);
    if (Vec2Dot(normal, polygon[minExtreme] - p0) > 0.0f)
        return false;

    // Test separating along normal vector of polygon[i] - polygon[i-1].
    Vector2 v0 = polygon[count - 1];
    Vector2 v1;
    for (int i = 0; i < count; i++)
    {
        v1 = polygon[i];
        dir = v1 - v0;
        normal.x = dir.y;
        normal.y = -dir.x;

        // Project line segment to get interval.
        float value = Vec2Dot(p0 - v0, normal);
        if (value <= 0.0f)
        {
            v0 = v1;
            continue;
        }
        value = Vec2Dot(p1 - v0, normal);
        if (value <= 0.0f)
        {
            v0 = v1;
            continue;
        }
        return false;
    }    

    return true;
}

/// <description>
/// SAT based on GJK algorithm.
/// </description>
bool SegmentPolygonIntersection_GJKBasedSAT(Vector2 p0, Vector2 p1, const Vector2* polygon, int count)
{
    GJK gjk;

    // Get the initial direction with any vertex of the minkowski body (i.e. the minkowski difference of the 
    // polygon and the line segment).
    Vector2 d = polygon[0] - p0;

    do 
    {
#if GJK_PROFILE
        counter_loop_GJK++;
#endif

        // Find p the support point of the polygon in -d, and q the support point of the line segment in d.
        // So s = p - q is the support point of the minkowski-body.
        const Vector2& p = polygon[FindMaximum(polygon, count, -d)];
        const Vector2& q = Vec2Dot(p0, d) >= Vec2Dot(p1, d) ? p0 : p1;
        Vector2 s = p - q;

        // If the origin is separated from the minkowski-body, no intersection.
        if (Vec2Dot(s, d) > 0.0f)
        {
            return false;
        }

        // Add the new support point.
        gjk.AddVertex(s);

        // Update the searching direction and simplex set.
        if (!gjk.Closest(d))
        {
            return true;
        }

    } while (true);

    return true;
}

/// <description>
/// SAT based on GJK algorithm.
/// </description>
bool SegmentPolygonIntersection_GJKBasedSATEx(Vector2 p0, Vector2 p1, const Vector2* polygon, int count)
{
    // Get the first point for the simplex.
    // Use the inward normal vector of the first edge of the polygon as the initial direction;
    // so, polygon[0] is the negative extream points for this direction.
#if GJK_PROFILE
    counter_loop_GJKEx++;
#endif
    Vector2 d(polygon[0].y - polygon[1].y, polygon[1].x - polygon[0].x);
    Vector2 s = polygon[0] - (Vec2Dot(p0 - p1, d) >= 0 ? p0 : p1);
    if (Vec2Dot(s, d) > 0.0f)
        return false;

    // Get the second point for the simplex.
#if GJK_PROFILE
    counter_loop_GJKEx++;
#endif
    d = s;
    s = Support(p0, p1, polygon, count, d);
    if (Vec2Dot(s, d) > 0.0f)
        return false;

    // Get the direction based on the simplex (line segment).
    // The direction is the normal vector towards a0-a1; if a0, a1 and the origin is ordered CW, 
    // then the normal vetor points to the origin, and set winding -1 to reverse it. else if CCW,
    // set winding 1. If a0, a1 and origin are colinear, then the origin is inside the minkowski 
    // body, intersect is true.
    Vector2 a0 = d;
    Vector2 a1 = s;
    Vector2 n(a0.y, -a0.x);    
    float sign = Vec2Dot(n, a1);
    if (fabs(sign) < EPSILON)    
        return false;
    float winding = sign > 0 ? -1.0f : 1.0f;
    d = winding * Vector2(a1.y - a0.y, a0.x - a1.x);

    // Continue updating the simplex.
    while (true)
    {
#if GJK_PROFILE
        counter_loop_GJKEx++;
#endif
        
        // Find the new support point.
        s = Support(p0, p1, polygon, count, d);
        if (Vec2Dot(s, d) > 0.0f)        
            return false;        

        // Check whether the origin is outside a2-a0.
        Vector2 a2 = s;
        Vector2 n1(a0.y - a2.y, a2.x - a0.x);        
        if (Vec2Dot(a2, n1) * winding < 0.0f)
        {
            a1 = a0;
            a0 = a2;

            winding = -winding;
            d = winding * n1;

            continue;
        }
        
        // Check whether the origin is outside a1-a2.
        Vector2 n2(a2.y - a1.y, a1.x - a2.x);
        if (Vec2Dot(a2, n2) * winding < 0.0f)
        {
            a0 = a1;
            a1 = a2;

            winding = -winding;
            d = winding * n2;

            continue;
        }

        // The origin is inside the triangle (a0, a1, a2), so intersect is true.
        return true;
    }

    // Assert(false). Should not get here.
    return true;
}

/// <description>
/// GJK based raycast.
/// </description>
bool SegmentPolygonIntersection_GJKBasedRayCast(Vector2 p0, Vector2 p1, const Vector2* polygon, int count)
{
    VoronoiSimplexSolver simplexSolver;
    
    // Relative motion vector. Motion A (the line segment) is p1 - p0, Motion B (the polygon) is 0.
	Vector2 r = p1 - p0;
    Vector2 v = r;
    float dist2 = Vec2LengthSquare(v);

    Vector2 p(0.0f, 0.0f);
    float lambda = 0.0f;

    int iter = MAX_ITERATIONS;

    while ((dist2 > EPSILON) && iter--)
    {
#if GJK_PROFILE
        counter_loop_RayCast++;
#endif
        // minkowski(B-A)
        Vector2 x = Support(p0, polygon, count, v);
        Vector2 w = p - x;

        float VdotW = Vec2Dot(v, w);
        if (VdotW < 0)
        {
            float VdotR = Vec2Dot(v, r);

            if (VdotR < EPSILON)
            {
                return false;
            }
            else
            {
                lambda -= VdotW / VdotR;
                if (lambda > 1.0f)
                {
                    return false;
                }

                p = lambda * r;
            }
        }

        if (!simplexSolver.InSimplex(x))
            simplexSolver.AddVertex(x);

        if (simplexSolver.Closest(v, p))
        {
            v -= p;
            dist2 = Vec2LengthSquare(v);
        }
        else
        {
            // p is inside the simplex, intersection is true.
            dist2 = 0.0f;
        }
    }
    
    return iter > 0;
}


/// <description>
/// First, divide polygon into two chain collected by minimum and maximum extreme points along normal vector
/// of the given segment. Then, do binary search for each chain to find the exact one edge that probably intersects
/// the line segment.
/// </description>
bool SegmentPolygonIntersection_BinarySearchByExtremes(Vector2 p0, Vector2 p1, const Vector2* polygon, int count)
{
    Vector2 dir = p1 - p0;
    Vector2 normal(dir.y, -dir.x);

    // Find minimum and maximum extremes.
    int minExtreme = FindMaximum(polygon, count, -normal);
    int maxExtreme = FindMaximum(polygon, count, normal);

    // If min and max extremes are both on one side of the given line segment, reject.
    float fMin = Vec2Dot(normal, polygon[minExtreme] - p0);
    float fMax = Vec2Dot(normal, polygon[maxExtreme] - p0);
    if (fMin * fMax > 0.0f)
        return false;

    // BinarySearch chain1 (extreme0, extreme1).
    int idx0, idx1;
    ChainBinarySearch(polygon, count, minExtreme, maxExtreme, normal, p0, 1.0f, idx0, idx1);
    Vector2 v0 = polygon[idx0];
    Vector2 v1 = polygon[idx1];
    Vector2 dvec = v1 - v0;
    Vector2 nvec(dvec.y, -dvec.x);
    float pp0 = Vec2Dot(nvec, p0 - v0);
    float pp1 = Vec2Dot(nvec, p1 - v0);
    if (pp0 > 0.0f && pp1 > 0.0f)
        return false;
    else if (pp0 * pp1 <= 0.0f)
        return true;

    // BinarySearch, chain1 (extreme1, extreme0).
    ChainBinarySearch(polygon, count, maxExtreme, minExtreme, normal, p0, -1.0f, idx0, idx1);
    v0 = polygon[idx0];
    v1 = polygon[idx1];
    dvec = v1 - v0;
    nvec.x = dvec.y;
    nvec.y = -dvec.x;
    pp0 = Vec2Dot(nvec, p0 - v0);
    pp1 = Vec2Dot(nvec, p1 - v0);
    if (pp0 > 0 && pp1 > 0)
        return false;

    return true;
}

/// <description>
/// This algorithm use binary search to find the exact 2 edges of the polygon which can have intersections 
/// with the given line segment. Then, do 2 times segment-segment-intersection test.
/// </description>
bool SegmentPolygonIntersection_BinarySearch(Vector2 p0, Vector2 p1, const Vector2* polygon, int count)
{
    // Get the direction vector and normal vector of the segment.
    Vector2 dir = p1 - p0;
    Vector2 normal(dir.y, -dir.x);
    
    // Test which side of the line segment the first vertex of the polygon is on.
    // If positive side, reverse the direction of normal vector to let the vertex on the negative side.
    float f = Vec2Dot(normal, polygon[0] - p0);
    if (f > 0)
    {
        f = -f;
        normal = -normal;
    }

    // Check wheter the front is the maximum along normal vector, if true, no intersection and stop.
    float pf = Vec2Dot(normal, polygon[0] - polygon[count - 1]);
    float nf = Vec2Dot(normal, polygon[1] - polygon[0]);
    if (f < 0.0f)
    {
        if ((pf > 0.0f && nf <= 0.0f) || (pf >= 0.0f && nf < 0.0f))
        {
            return false;
        }
    }
    else if (f == 0.0f)
    {
        // If polygon[0] is exact on the infinite line of (p0, p1) and is maximum, 
        // reverse the normal vector and let it to the minimum; this is for consistent operation later on.
        if ((pf > 0.0f && nf <= 0.0f) || (pf >= 0.0f && nf < 0.0f))
        {
            normal = -normal;
        }
    }

    // Binary Search.
    int front = 0;
    int end = 0;
    int middle = 0;
    while (true)
    {
        // Get middle.
        if (front < end)
            middle = (front + end) / 2;
        else
            middle = ((front + end + count) / 2) % count;

        // Test which side of the line segment the middle vertex of the polygon is on.
        float m = Vec2Dot(normal, polygon[middle] - p0);
        if (m >= 0)
        {
            // Two chains are found, break.
            break;
        }

        // Check whether the middle point is maximum point along normal.
        // If true, the line segment cannot intersect the polygon.
        float pm = Vec2Dot(normal, polygon[middle] - polygon[(middle - 1 + count) % count]);
        float nm = Vec2Dot(normal, polygon[(middle + 1) % count] - polygon[middle]);
        if ((pm > 0 && nm <= 0) || (pm >= 0 && nm < 0))
        {
            return false;
        }

        // Choose half chain.
        if (nf > 0)
        {
            if (nm < 0)
            {
                end = middle;
            }
            else
            {
                float k = Vec2Dot(normal, polygon[front] - polygon[middle]);
                if (k > 0)
                {
                    end = middle;
                }
                else
                {
                    front = middle;
                    nf = nm;
                }
            }
        }
        else
        {
            if (nm > 0)
            {
                front = middle;
                nf = nm;
            }
            else
            {
                float k = Vec2Dot(normal, polygon[front] - polygon[middle]);
                if (k > 0)
                {
                    front = middle;
                    nf = nm;
                }
                else
                {
                    end = middle;
                }
            }
        }
    };

    // BinarySearch (front, middle)
    // front <= 0, middle >= 0(but not == 0 simultaneously). 
    int idx0 = 0;
    int idx1 = 0;
    ChainBinarySearch(polygon, count, front, middle, normal, p0, 1.0f, idx0, idx1);
    // Find the edge (v0, v1), v0, v1 are on the different sides of infinite line (p0, p1).
    Vector2 v0 = polygon[idx0];
    Vector2 v1 = polygon[idx1];
    Vector2 dvec = v1 - v0;
    Vector2 nvec(dvec.y, -dvec.x);
    // If p0, p1 is both out of the edge (v0, v1), return false.
    // Else if intersects, return true.
    // Else, goto the test for the other edge.
    float pp0 = Vec2Dot(nvec, p0 - v0);
    float pp1 = Vec2Dot(nvec, p1 - v0);
    if (pp0 > 0.0f && pp1 > 0.0f)
        return false;
    else if (pp0 * pp1 <= 0.0f)
        return true;

#ifdef _ENDPOINTS_OUTSIDE_
    return false;
#else
    // BinarySearch (middle, end)
    // middle >= 0, end <= 0(but not = 0 simultaneously).
    ChainBinarySearch(polygon, count, middle, end, normal, p0, -1.0f, idx0, idx1);
    // Find the edge (v0, v1), v0, v1 are on the different sides of infinite line (p0, p1).
    v0 = polygon[idx0];
    v1 = polygon[idx1];
    dvec = v1 - v0;
    nvec.x = dvec.y;
    nvec.y = -dvec.x;
    // If p0, p1 is both out of the edge (v0, v1), return false.
    // Else, return true.
    pp0 = Vec2Dot(nvec, p0 - v0);
    pp1 = Vec2Dot(nvec, p1 - v0);
    if (pp0 > 0 && pp1 > 0)
        return false;

    return true;
#endif
}

/// <description>
/// Line segment to convex polygon test by doing point-in-polygon first.
/// </description>
bool SegmentPolygonIntersection_ExtremePlusPointInside(Vector2 p0, Vector2 p1, const Vector2* polygon, int count)
{
#ifndef _ENDPOINTS_OUTSIDE_
    // First, test whether p0 or p1 is inside the polygon. If true, stop.
    if (PointInPolygon_CrossingsConvex(p0, polygon, count) ||
        PointInPolygon_CrossingsConvex(p1, polygon, count))
        return true;
#endif

    // If the two points are both outside, find the 2 extreme points of the polygon along
    // normal direction of the line segment.
    Vector2 dir = p1 - p0;
    Vector2 normal(dir.y, -dir.x);
    int minExtreme = FindMaximum(polygon, count, -normal);
    int maxExtreme = FindMaximum(polygon, count, normal);

    // Test whether (p0, p1) intersects (minExtreme, maxExtreme).
    return SegmentSegmentIntersection_Antonio(p0, p1, polygon[minExtreme], polygon[maxExtreme]);    
}

/// <description>
/// LIFO(Last-In-First-Out) algorithm.
/// Treat edges of the convex polygon as half-planes. Compute intersections between the line segment
/// and the half-planes, only when last entering is before first leaving, intersection is got.
/// </description>
bool SegmentPolygonIntersection_LIFO(Vector2 p0, Vector2 p1, const Vector2* polygon, int count)
{
    Vector2 dir = p1 - p0;

    float tEnter = 0.0f;
    float tLeave = 1.0f;
    Vector2 v0 = polygon[count - 1];
    Vector2 v1(0.0f, 0.0f);
    for (int i = 0; i < count; i++)
    {
        v1 = polygon[i];
        Vector2 edge = v1 - v0;
        Vector2 normal(edge.y, -edge.x);

        float f = Vec2Dot(normal, dir);
        float m = Vec2Dot(normal, v0 - p0);
        if (fabs(f) < EPSILON)
        {
            // If (p0, p1) is parellel with (v0, v1), and outside half plane, return false.
            if (m < 0.0f)
                return false;
            else
            {
                v0 = v1;
                continue;
            }
        }
        else
        {
            // Compute intersection, and update last entering or first leaving.
            // If last entering > first leaving, return false.
            float t = m / f;
            if (f < 0.0f)
            {
                // Entering.
                if (t > tEnter)
                    tEnter = t;
                if (tEnter > tLeave)
                    return false;
            }
            else
            {
                // Leaving.
                if (t < tLeave)
                    tLeave = t;
                if (tLeave < tEnter)
                    return false;
            }
            v0 = v1;
        }
    }
    
    return true;
}

/// <description>
/// Antonio algorithm to solve 2 line segments intersection.
/// </description>
bool SegmentSegmentIntersection_Antonio_Direction(Vector2 start0, Vector2 d0, Vector2 start1, Vector2 d1)
{    
    Vector2 n0(d0.y, -d0.x);    
    Vector2 n1(d1.y, -d1.x);

    // If parallel, rejet.
    float f = Vec2Dot(d0, n1);
    if (abs(f) < EPSILON)
    {
        return false;
    }
    bool sign = (f > 0.0f);

    // Test whether (s0, e0) intersects the infinite line (s1, e1).
    Vector2 diff = start1 - start0;
    float m1 = Vec2Dot(diff, n1);
    if (sign)
    {
        if (m1 < 0.0f || m1 > f)
        {
            return false;
        }
    }
    else if (m1 > 0.0f || m1 < f)
    {
        return false;
    }

    // Test whether (s1, e1) intersects the infinite line (s0, e0).
    float m2 = Vec2Dot(diff, n0);
    if (sign)
    {
        if (m2 < 0.0f || m2 > f)
        {
            return false;
        }
    }
    else if (m2 > 0.0f || m2 < f)
    {
        return false;
    }
    
    return true;
}

/// <description>
/// Antonio algorithm to solve 2 line segments intersection.
/// </description>
bool SegmentSegmentIntersection_Antonio(Vector2 start0, Vector2 end0, Vector2 start1, Vector2 end1)
{    
    Vector2 d0 = end0 - start0;    
    Vector2 d1 = end1 - start1;
    
    return SegmentSegmentIntersection_Antonio_Direction(start0, d0, start1, d1);
}

/// <description>
/// Based on Antonio algirthm, but do early rejection by checking separating along x/y axis.
/// </description>
bool SegmentSegmentIntersection_EarlyReject(Vector2 start0, Vector2 end0, Vector2 start1, Vector2 end1)
{
    // Separating test along x axis.
    float min0 = start0.x;
    float max0 = end0.x;
    MinMax(min0, max0);
    float min1 = start1.x;
    float max1 = end1.x;
    MinMax(min1, max1);
    if (min0 > max1 || min1 > max0)
    {
        return false;
    }

    // Separating test along y axis.
    min0 = start0.y;
    max0 = end0.y;
    MinMax(min0, max0);
    min1 = start1.y;
    max1 = end1.y;
    MinMax(min1, max1);
    if (min0 > max1 || min1 > max0)
    {
        return false;
    }

    return SegmentSegmentIntersection_Antonio(start0, end0, start1, end1);
}

/// <description>
/// Basic crossings test to test point-in-polygon.
/// </description>
bool PointInPolygon_Crossings(Vector2 point, const Vector2* polygon, int count)
{        
    Vector2 p0 = polygon[count - 1];
    Vector2 p1(0.0f, 0.0f);
    bool f0 = (p0.y >= point.y);
    bool f1 = true;
    bool inside = false;
    
    for (int i = 0; i < count; i++)
    {
        p1 = polygon[i];
        f1 = (p1.y >= point.y);
        if (f0 != f1)
        {
            if (f0 == 
                ((point.x - p0.x) * (p1.y - p0.y) + (point.y - p0.y) * (p0.x - p1.x) > 0))
            {
                inside = !inside;
            }
        }
        f0 = f1;
        p0 = p1;
    }
    
    return inside;
}

/// <description>
/// Binary search.
/// From polygon[front] to polygon[end], find p and q if polygon[p] and polygon[q] are at different sides of dir vector.
/// The method assumes that polygon[front] < polygon[end], if not, the input sign should be negative value.
/// </description>
void ChainBinarySearch(const Vector2* polygon, int count, int front, int end, Vector2 dir, Vector2 pivot, float sign, int& p, int& q)
{
    int middle = 0;
    while (true)
    {
        if ((end - front + count) % count == 1)
            break;
        
        // Get middle.
        if (front < end)
        {
            middle = (front + end) / 2;
        }
        else
        {
            middle = ((front + end + count) / 2) % count;
        }

        // Calculate projection and choose half chain.
        float m = sign * Vec2Dot(dir, polygon[middle] - pivot);
        if (m > 0)
            end = middle;
        else
            front = middle;
    };

    p = front;
    q = end;
}

/// <description>
/// Project polygon to given vector and compute interval.
/// </description>
void ComputeInterval(const Vector2* polygon, int count, Vector2 dir, Vector2 pivot, float& min, float& max)
{
    min = max = Vec2Dot(polygon[0] - pivot, dir);
    for (int i = 1; i < count; i++)
    {
        float value = Vec2Dot(polygon[i] - pivot, dir);
        if (value < min)
        {
            min = value;
        }
        else if (value > max)
        {
            max = value;
        }
    }
}

/// <description>
/// Project polygon to given vector and see whether the interval strides 0.
/// </description>
int ComputeInterval_Optimized(const Vector2* polygon, int count, Vector2 dir, Vector2 pivot)
{
    bool pos = false;
    bool neg = false;
    for (int i = 0; i < count; i++)
    {
        float value = Vec2Dot(polygon[i] - pivot, dir);
        if (value > 0.0f)
        {
            pos = true;
        }
        else if (value < 0.0f)
        {
            neg = true;
        }
        else
        {
            return 0;
        }

        if (pos && neg)
        {
            return 0;
        }
    }

    if (pos)
    {
        return 1;
    }
    else
    {
        return -1;
    }
}

/// <description>
/// Find maximum extreme point of the polygon according to given direction.
/// </description>
int FindMaximum(const Vector2* polygon, int count, Vector2 dir)
{
    float pf = Vec2Dot(dir, polygon[0] - polygon[count - 1]);
    float nf = Vec2Dot(dir, polygon[1] - polygon[0]);
    if (pf > 0 && nf <= 0 || pf >= 0 && nf < 0)
        return 0;

    int front = 0;
#if _ACT_AS_UNCLOSED_
    int rear = count;
#else
    int rear = 0;
#endif

    while (true)
    {
        int middle;

#if _ACT_AS_UNCLOSED_
        middle = (front + rear) / 2;
#else
        if (front < rear)
            middle = (front + rear) / 2;
        else
            middle = ((front + rear + count) / 2) % count;
#endif

#if _ACT_AS_UNCLOSED_
        float pm = Vec2Dot(dir, polygon[middle] - polygon[middle - 1]);
        float nm = Vec2Dot(dir, polygon[middle + 1] - polygon[middle]);
#else
        float pm = Vec2Dot(dir, polygon[middle] - polygon[(middle - 1 + count) % count]);
        float nm = Vec2Dot(dir, polygon[(middle + 1) % count] - polygon[middle]);
#endif
        if (pm > 0 && nm <= 0 || pm >= 0 && nm < 0)
            return middle;

        if (nf > 0)
        {
            if (nm < 0)
            {
                rear = middle;
            }
            else
            {
                float k = Vec2Dot(dir, polygon[front] - polygon[middle]);
                if (k > 0)
                {
                    rear = middle;
                }
                else
                {
                    front = middle;
                    nf = nm;
                }
            }
        }
        else
        {
            if (nm > 0)
            {
                front = middle;
                nf = nm;
            }
            else
            {
                float k = Vec2Dot(dir, polygon[front] - polygon[middle]);
                if (k > 0)
                {
                    front = middle;
                    nf = nm;
                }
                else
                {
                    rear = middle;
                }
            }
        }
    };

    return false;
}

/// <description>
/// Crossings test for convex polygon. 
/// Stop when find the 2 possible crossings.
/// </description>
bool PointInPolygon_CrossingsConvex(Vector2 point, const Vector2* polygon, int count)
{
    Vector2 p0 = polygon[count - 1];
    Vector2 p1(0.0f, 0.0f);
    bool f0 = (p0.y >= point.y);
    bool f1 = true;
    bool inside = false;
    int crossings = 0;
    
    for (int i = 0; i < count && crossings < 2; i++)
    {
        p1 = polygon[i];
        f1 = (p1.y >= point.y);
        if (f0 != f1)
        {
            crossings++;
            
            if (f0 == 
                ((point.x - p0.x) * (p1.y - p0.y) + (point.y - p0.y) * (p0.x - p1.x) > 0))
            {
                inside = !inside;
            }
        }
        f0 = f1;
        p0 = p1;
    }
    
    return inside;    
}

/// <description>
/// Clip line segment by the box.
/// TODO: refer to the algorithm in GeometricTools to get a better solution.
/// </description>
bool ClipSegment(const Vector2& p0, const Vector2& p1, 
				 const Vector2& minCorner, const Vector2& maxCorner, 
				 Vector2& pa, Vector2& pb)
{
	Vector2 direction = p1 - p0;
	
	float tMin = -INFINITE_VALUE;
	float tMax = INFINITE_VALUE;
	
	// Intersections on X-slab.
	if (!LineSlabIntersect(p0.x, direction.x, minCorner.x, maxCorner.x, tMin, tMax))
		return false;

	// Intersection on Y-slab.
	if (!LineSlabIntersect(p0.y, direction.y, minCorner.y, maxCorner.y, tMin, tMax))
		return false;

	// Clip to 0~1.
	if (tMin > 1.0f || tMax < 0.0f)
		return false;

	// Get the endpoints after clipping.
    pa = tMin < 0.0f ? p0 : p0 + tMin * direction;
    pb = tMax > 1.0f ? p1 : p0 + tMax * direction;
	return true;
}

/// <description>
/// Line-Slab intersection method.
/// </description>
bool LineSlabIntersect(float start, float direction, float minSide, float maxSide, float& tMin, float& tMax)
{
	// Parallel to the slab.
	if (fabs(direction) < EPSILON)
	{
		if (start < minSide || start > maxSide)
			return false;
	}
	
	// Non-parallel, get parameters for 2 intersections.
	float invDirection = 1.0f / direction;
	float t1 = (minSide - start) * invDirection;
	float t2 = (maxSide - start) * invDirection;
	if (t1 > t2)
		Swap(t1, t2);
	
	// Update tMin, tMax.
	if (t1 > tMin)
		tMin = t1;
	if (t2 < tMax)
		tMax = t2;

	// Check tMin, tMax.
	if (tMin > tMax)
		return false;
	else
		return true;
}

/// <description>
/// Find the bounding box of the given line segment.
/// </description>
void BoundingBoxOfSegment(const Vector2& p0, const Vector2& p1, const Vector2& dir, Vector2& minCorner, Vector2& maxCorner)
{
	if (dir.x >= 0)
	{
		minCorner.x = p0.x;
		maxCorner.x = p1.x;
	}
	else
	{
		minCorner.x = p1.x;
		maxCorner.x = p0.x;
	}

	if (dir.y >= 0)
	{
		minCorner.y = p0.y;
		maxCorner.y = p1.y;
	}
	else
	{
		minCorner.y = p1.y;
		maxCorner.y = p0.y;
	}
}

/// <description>
/// Box-box intersection test.
/// </description>
bool BoxBoxIntersect(const Vector2& min0, const Vector2& max0, const Vector2& min1, const Vector2& max1)
{
	if ( (min0.x > max1.x || max0.x < min1.x) ||
		 (min0.y > max1.y || max0.y < min1.y) )
		 return false;
	return true;
}

/// <description>
/// Find the support point on minkowski difference between the polygon and the line segment in -direction.
/// </description>
Vector2 Support(const Vector2& p0, const Vector2& p1, const Vector2* polygon, int count, const Vector2& direction)
{
    const Vector2& p = polygon[FindMaximum(polygon, count, -direction)];
    const Vector2& q = Vec2Dot(p0, direction) >= Vec2Dot(p1, direction) ? p0 : p1;
    return p - q;
}

/// <description>
/// Find the support point on minkowski difference between the polygon and the point in -direction.
/// </description>
Vector2 Support(const Vector2& point, const Vector2* polygon, int count, const Vector2& direction)
{
    const Vector2& p = polygon[FindMaximum(polygon, count, -direction)];
    return p - point;
}