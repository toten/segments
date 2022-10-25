#ifndef _BENTLEY_OTTMANN_ACCELERATED_H_
#define _BENTLEY_OTTMANN_ACCELERATED_H_

#include "Algorithm.h"
#include "BentleyOttmann.h"

/// <description>
/// Helper class for grid-accelerated segment-polygon test.
/// </description>
class BentleyOttmannHelper
{
protected:
	/// <description>
	/// Hold the pointer to the current polygon.
	/// </description>
	const Vector2* mCurrentPolygon;

	/// <description>
	/// BentleyOttmann acceleration structure associated to current polygon.
	/// </description>
	BentleyOttmann* mBentleyOttmann;

public:
	/// <description>
	/// Constructor.
	/// </description>
	BentleyOttmannHelper();

	/// <description>
	/// Destructor.
	/// </description>
	~BentleyOttmannHelper();

public:
	/// <description>
	/// Do grid-accelerated segment-polygon test.
	/// </description>
	bool DoSegmentPolygonTest(const Vector2& p0, const Vector2& p1, const Vector2* polygon, int count);

protected:
	/// <description>
	/// Prepare grid acceleration structure for current polygon.
	/// </description>
	bool PrepareBentleyOttmann(const Vector2* polygon, int count);
};

/// <description>
/// BentleyOttmann-accelerated segment-polygon test algorithm.
/// </description>
bool SegmentPolygonIntersection_BentleyOttmannAccelerated(Vector2 p0, Vector2 p1, const Vector2* polygon, int count);

#endif