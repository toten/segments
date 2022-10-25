#ifndef _BENTLEY_OTTMANN_H_
#define _BENTLEY_OTTMANN_H_

#include "Math.h"
#include "Array.h"
#include "BitSet.h"

/// <description>
/// Event type enumeration.
/// </description>
enum EEventType
{
	// Undefined even type.
    Undefined   = 0,
    
	// Insert two segments.
    Insert      = 1,
    
	// Replace one segment with another.
    Replace     = 2,
    
	// Remove two segments.
    Remove      = 3,
};

/// <description>
/// Segment structure.
/// </description>
struct Segment
{
	// Left vertex.
    Vector2 leftVertex;

	// Right vertex.
    Vector2 rightVertex;

	// Delta.
	Vector2 delta;

	/// <description>
	/// 
	/// </description>
	static bool IsPointBelowSegment(const Vector2& point, const Segment& segment)
	{
		return segment.delta.y * (point.x - segment.leftVertex.x) - 
			segment.delta.x * (point.y - segment.leftVertex.y) > 0;
	}

	/// <description>
	/// 
	/// </description>
	static bool IsSegmentIntersect(const Segment& s0, const Segment& s1, bool isBelow)
	{
		if (s0.rightVertex.x < s1.rightVertex.x)
		{
			if (Segment::IsPointBelowSegment(s0.rightVertex, s1) == isBelow)
				return false;
			else
				return true;
		}
		else
		{
			if (Segment::IsPointBelowSegment(s1.rightVertex, s0) != isBelow)
				return false;
			else
				return true;
		}
	}
};

/// <description>
/// Event structure.
/// </description>
struct Event
{
	// Vertex.
    Vector2			vertex;
    
	// Event type.
    EEventType		eventType;

	// Operation opsition in the segment list of this event.
    int				operationPosition;

	// Segment list represented by the indices.
    Array<int>		segmentIndexList;
};


/// <description>
/// BentleyOttmann class.
/// </description>
class BentleyOttmann
{
protected:
	// Vertex array of the polygon.
    const Vector2*              mVertices;

	// Vertex count.
	int                         mCount;
    
    // Event list.
    Event*                      mEventList;

    // Segment list.
    Segment*                    mSegments;

	// Segment tested flags.
	BitSet						mSegmentTestFlags;

public:
	/// <description>
	///
	/// </description>
	static BentleyOttmann* CreateBentleyOttmann(const Vector2* polygon, int count);

	/// <description>
	///
	/// </description>
	static void ReleaseBentleyOttmann(BentleyOttmann* value);

protected:
	/// <description>
	/// Constructor.
	/// </description>
	BentleyOttmann(const Vector2* polygon, int count): mCount(count), mVertices(polygon), mEventList(NULL), mSegments(NULL) {}

	/// <description>
	///
	/// </description>
	~BentleyOttmann() 
	{
		if (mEventList)
			delete [] mEventList;

		if (mSegments)
			delete [] mSegments;
	}

	/// <description>
	///
	/// </description>
	bool Initialize();

public:
	/// <description>
	///
	/// </description>
	bool IsPointInPolygon(const Vector2& point);

	/// <description>
	///
	/// </description>
    bool IsSegmentIntersectPolygon(const Vector2& p0, const Vector2& p1);

protected:
	/// <description>
	///
	/// </description>
	void QuickSort(int start, int end, int arr[]);
};

#endif