#include "Common.h"

#include "BentleyOttmannAccelerated.h"

// Global helper.
BentleyOttmannHelper g_BentleyOttmannHelper;

// Constructor.
BentleyOttmannHelper::BentleyOttmannHelper() : mCurrentPolygon(NULL), mBentleyOttmann(NULL) {}

// Destructor.
BentleyOttmannHelper::~BentleyOttmannHelper()
{
	if (mBentleyOttmann)
	{
		BentleyOttmann::ReleaseBentleyOttmann(mBentleyOttmann);
		mBentleyOttmann = NULL;
	}
}

// Do grid-accelerated segment-polygon test.
bool BentleyOttmannHelper::DoSegmentPolygonTest(const Vector2& p0, const Vector2& p1, const Vector2* polygon, int count)
{
	// Prepare grid acceleration structure for current polygon.
	if (!PrepareBentleyOttmann(polygon, count))
		return false;

	// Do the test.
#ifndef _ENDPOINTS_OUTSIDE_
	// Firstly, check whether 2 endpoints of the segment is inside the polygon.
	if (mBentleyOttmann->IsPointInPolygon(p0) ||
		mBentleyOttmann->IsPointInPolygon(p1))
		return true;
#endif

	// If none, check whether the segment intersect the polygon.
	return mBentleyOttmann->IsSegmentIntersectPolygon(p0, p1);
}

// Prepare grid acceleration structure for current polygon.
bool BentleyOttmannHelper::PrepareBentleyOttmann(const Vector2* polygon, int count)
{
	if (mCurrentPolygon != polygon)
	{
		// Release old grid structure.
        if (mBentleyOttmann != NULL)
            BentleyOttmann::ReleaseBentleyOttmann(mBentleyOttmann);

		// Create new grid structure.
		mBentleyOttmann = BentleyOttmann::CreateBentleyOttmann(polygon, count);
		if (mBentleyOttmann == NULL)
			return false;
		
		// Update.
		mCurrentPolygon = polygon;
	}

	return true;
}

/// <description>
/// BentleyOttmann-accelerated segment-polygon test algorithm.
/// </description>
bool SegmentPolygonIntersection_BentleyOttmannAccelerated(Vector2 p0, Vector2 p1, const Vector2* polygon, int count)
{
	return g_BentleyOttmannHelper.DoSegmentPolygonTest(p0, p1, polygon, count);
}