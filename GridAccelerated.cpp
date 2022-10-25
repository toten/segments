#include "GridAccelerated.h"

// Global helper.
GridHelper g_GridHelper;

// Constructor.
GridHelper::GridHelper() : mCurrentPolygon(NULL), mGrid(NULL) {}

// Destructor.
GridHelper::~GridHelper()
{
	if (mGrid)
	{
		Grid::DestroyGrid(mGrid);
		mGrid = NULL;
	}
}

// Do grid-accelerated segment-polygon test.
bool GridHelper::DoSegmentPolygonTest(const Vector2& p0, const Vector2& p1, const Vector2* polygon, int count)
{
	// Prepare grid acceleration structure for current polygon.
	if (!PrepareGrid(polygon, count))
		return false;

	// Do the test.
#ifndef _ENDPOINTS_OUTSIDE_
	// Firstly, check whether 2 endpoints of the segment is inside the polygon.
	if (mGrid->IsPointInPolygon(p0) ||
		mGrid->IsPointInPolygon(p1))
		return true;
#endif

	// If none, check whether the segment intersect the polygon.
	return mGrid->IsSegmentIntersectPolygon(p0, p1);
}

// Prepare grid acceleration structure for current polygon.
bool GridHelper::PrepareGrid(const Vector2* polygon, int count)
{
	if (mCurrentPolygon != polygon)
	{
		// Release old grid structure.
        if (mGrid != NULL)
            Grid::DestroyGrid(mGrid);

		// Create new grid structure.
		mGrid = Grid::CreateGrid(polygon, count);
		if (mGrid == NULL)
			return false;
		
		// Update.
		mCurrentPolygon = polygon;
	}

	return true;
}

/// <description>
/// Grid-accelerated segment-polygon test algorithm.
/// </description>
bool SegmentPolygonIntersection_GridAccelerated(Vector2 p0, Vector2 p1, const Vector2* polygon, int count)
{
	return g_GridHelper.DoSegmentPolygonTest(p0, p1, polygon, count);
}