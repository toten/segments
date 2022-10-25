#ifndef _GRID_ACCELERATED_H_
#define _GRID_ACCELERATED_H_

#include "Algorithm.h"
#include "Grid.h"

/// <description>
/// Helper class for grid-accelerated segment-polygon test.
/// </description>
class GridHelper
{
protected:
	/// <description>
	/// Hold the pointer to the current polygon.
	/// </description>
	const Vector2* mCurrentPolygon;

	/// <description>
	/// Grid acceleration structure associated to current polygon.
	/// </description>
	Grid* mGrid;

public:
	/// <description>
	/// Constructor.
	/// </description>
	GridHelper();

	/// <description>
	/// Destructor.
	/// </description>
	~GridHelper();

public:
	/// <description>
	/// Do grid-accelerated segment-polygon test.
	/// </description>
	bool DoSegmentPolygonTest(const Vector2& p0, const Vector2& p1, const Vector2* polygon, int count);

protected:
	/// <description>
	/// Prepare grid acceleration structure for current polygon.
	/// </description>
	bool PrepareGrid(const Vector2* polygon, int count);
};

/// <description>
/// Grid-accelerated segment-polygon test algorithm.
/// </description>
bool SegmentPolygonIntersection_GridAccelerated(Vector2 p0, Vector2 p1, const Vector2* polygon, int count);

#endif