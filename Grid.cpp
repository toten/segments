#include "Common.h"

#include "Grid.h"
#include "Algorithm.h"

float Grid::EXPAND_INCREMENT        = 1e-5f;

int Grid::MAXIMUM_EDGE_NUMBER       = 8;

int Grid::MAXIMUM_RECREATE          = 5;

/// <description>
/// Constructor. Called internally.
/// </description>
Grid::Grid(const Vector2* polygon, int count) : mVertices(polygon)
    , mTotalEdgeCount(count)
	, mMinPolygon(0.0f, 0.0f)
	, mMaxPolygon(0.0f, 0.0f)
    , mMin(0.0f, 0.0f)
    , mMax(0.0f, 0.0f)
    , mRow(0)
    , mColumn(0)
    , mTotalCellCount(0)
    , mCellSize(0.0f, 0.0f)
    , mInvCellSize(0.0f, 0.0f)
    , mLongitudes(NULL)
    , mLatitudes(NULL)
    , mCells(NULL)
{
}

/// <description>
/// Destructor. Called internally.
/// </description>
Grid::~Grid()
{
    Clear();
}

/// <description>
/// Create a grid acceleration structure from the given polygon.
/// Return NULL if fail to create the structure.
/// </description>
Grid* Grid::CreateGrid(const Vector2* polygon, int count)
{
    Grid* pGrid = new Grid(polygon, count);

    if (!pGrid->Initialize())
    {
        delete pGrid;
        return NULL;
    }
    else
    {
        return pGrid;
    }
}

/// <description>
/// Destroy the grid structure.
/// </description>
void Grid::DestroyGrid(Grid* pGrid)
{
    delete pGrid;
    pGrid = NULL;
}

/// <description>
/// Initialize grid structure.
/// </description>
bool Grid::Initialize()
{
    // Count is too small to create a acceleration structure.
    if (mTotalEdgeCount <= MAXIMUM_EDGE_NUMBER)
        return false;

    // Get bounding box of the polygon.
    Vector2 minCorner, maxCorner;
    CalculateBoundingBox(mVertices, mTotalEdgeCount, mMinPolygon, mMaxPolygon);
	Vector2 diff = mMaxPolygon - mMinPolygon;

    int reCreate = 0;
    bool success = false;
    while (reCreate < MAXIMUM_RECREATE)
    {       
        mMin = mMinPolygon;
        mMax = mMaxPolygon;        

        // Expand a little to ensure everyting falls inside area.        
        mMin.x -= EXPAND_INCREMENT * diff.x;
        mMax.x += EXPAND_INCREMENT * diff.x;
        mMin.y -= EXPAND_INCREMENT * diff.y;
        mMax.y += EXPAND_INCREMENT * diff.y;

        success = Setup();
        if (success)
            break;

        reCreate++;
    };    

    return success;
}

/// <description>
/// Setup grid.
/// </description>
bool Grid::Setup()
{
    // Determine the sub-dividing strategy, i.e. mRow * mColumn.
    SubDivide();

    // Calculate cell size and the inverse.
    Vector2 diff = mMax - mMin;
    mCellSize.x = diff.x / mColumn;
    mCellSize.y = diff.y / mRow;
    mInvCellSize.x = 1.0f / mCellSize.x;
    mInvCellSize.y = 1.0f / mCellSize.y;
    
    // Save Coordinates for longtitudes and latitudes of the grid.
    mLongitudes = new float[mColumn + 1];
    mLatitudes = new float[mRow + 1];
    for (int i = 0; i < mColumn; ++i)
        mLongitudes[i] = mMin.x + mCellSize.x * i;
    mLongitudes[mColumn] = mMax.x;   // Make last grid corner precisely correct.
    for (int i = 0; i < mRow; ++i)
        mLatitudes[i] = mMin.y + mCellSize.y * i;
    mLatitudes[mRow] = mMax.y;      // Make last grid corner precisely correct.

    // Initialize cells.
    mCells = new Cell[mTotalCellCount];

    // Loop through edges and insert into grid structure.    
    if (!CategorizeEdge())        
    {
        // If fails, clearing the grid and return.
        Clear();
        return false;
    }

    // Check inclusion of corners of cells.
    CategorizeCellCorner();

    return true;
}

/// <description>
/// Divide the bounding box of the polygon into mColumn * mRow grid.
/// </description>
void Grid::SubDivide()
{
    int totalCount = 1 + (mTotalEdgeCount - 1) / MAXIMUM_EDGE_NUMBER;

    // Solve equation group:
    // (1) mColumn * mRow = totalCount;
    // (2) mColumn / mRow = dx / dy;
    Vector2 diff = mMax - mMin;
    mRow = int(sqrt(float(totalCount * (diff.y / diff.x))) + 0.5f);
    mColumn = int(mRow * (diff.x / diff.y) + 0.5f);

    // Recompute total count.
    mTotalCellCount = mRow * mColumn;
}

/// <description>
/// Loop through edges and insert into grid structure.
/// </description>
bool Grid::CategorizeEdge()
{
    const Vector2* pV0 = &mVertices[mTotalEdgeCount - 1];
    for (int i = 0; i < mTotalEdgeCount; ++i)
    {
        const Vector2* pV1 = &mVertices[i];

        // Insure va.y <= vb.y;
        Vector2 va, vb;
        if (pV0->y <= pV1->y)
        {
            va = *pV0;
            vb = *pV1;
        }
        else
        {
            va = *pV1;
            vb = *pV0;
        }

        if (!CategorizeEdge(va, vb))
        {
            return false;
        }
        else
        {
            pV0 = pV1;
        }
    }

    return true;
}

/// <description>
/// Deal with the given edge and intert it into grid structrue.
/// This method assumes that va.y <= vb.y.
/// </description>
bool Grid::CategorizeEdge(const Vector2& va, const Vector2& vb)
{
    // If va is same as vb, return.
    Vector2 dir = vb - va;
    float tMax = sqrt(dir.x * dir.x + dir.y * dir.y);
    if (tMax < EPSILON)
    {
        return true;
    }
    // Normalize direction vector.
    dir /= tMax;

    // Find the cell where va locates.
    int cx = int((va.x - mMin.x) * mInvCellSize.x);
    int cy = int((va.y - mMin.y) * mInvCellSize.y);

    // Find tx, ty, the t values of intersection points respectively on next vertical and horizontal lines.        
    float tx, ty;
    float dtx, dty;        
    Vector2 invDir;
    // Calculate tx.
    if (fabs(dir.x) < EPSILON)
    {
        tx = INFINITE_VALUE;            
    }
    else
    {
        invDir.x = 1.0f / dir.x;            
        if (dir.x > 0)
        {
            tx = (mLongitudes[cx + 1] - va.x) * invDir.x;
            dtx = mCellSize.x * invDir.x;   // Distance along the edge between two vertical lines.
        }
        else
        {
            tx = (mLongitudes[cx] - va.x) * invDir.x;
            dtx = -mCellSize.x * invDir.x;
        }            
    }
    // Calculate ty.
    if (dir.y < EPSILON)
    {            
        ty = INFINITE_VALUE;            
    }
    else
    {
        invDir.y = 1.0f / dir.y;            
        ty = (mLatitudes[cy + 1] - va.y) * invDir.y;
        dty = mCellSize.y * invDir.y;   // Distance along the edge between two horizontal lines.          
    }

    // Categorize edge (va + t * dir) to cells.
    Cell* pCell = &mCells[cy * mColumn + cx];
    Cell* pNextCell = NULL;
    float tNear = 0.0f;
    Vector2 _va = va;
    Vector2 _vb;
    while (tNear < tMax)
    {
        if (fabs(tx - ty) < EPSILON)
        {
            // Edge crosses the corner of the cell. Re-generate the grid.
            return false;
        }

        if (tx < ty)
        {
            // Hit vertical line first.                
            // Update tNear.
            tNear = tx;                

            if (tNear < tMax)
            {
                if (dir.x > 0)
                {
                    // Hit right side of this cell.
                    pCell->mFlag = (ECellFlag)(pCell->mFlag | CELL_R_EDGE_HIT);

                    // Get x,y coordinate of intersection point.
                    _vb.x = mLongitudes[cx + 1];
                    _vb.y = va.y + tNear * dir.y;

                    // Get next cell.
                    ++cx;
                    pNextCell = &mCells[cy * mColumn + cx];

                    // Hit left side of the next cell.
                    pNextCell->mFlag = (ECellFlag)(pNextCell->mFlag | CELL_L_EDGE_HIT);
                }
                else
                {
                    // Hit left side of this cell.
                    pCell->mFlag = (ECellFlag)(pCell->mFlag | CELL_L_EDGE_HIT);

                    // Get x,y coordinate of intersection point.
                    _vb.x = mLongitudes[cx];
                    _vb.y = va.y + tNear * dir.y;

                    // Get next cell.
                    --cx;
                    pNextCell = &mCells[cy * mColumn + cx];

                    // Hit right side of the next cell.
                    pNextCell->mFlag = (ECellFlag)(pNextCell->mFlag | CELL_R_EDGE_HIT);
                }                    
            }
            else
            {
                _vb = vb;
            }

            // Update tx.
            tx += dtx;
        }
        else
        {
            // Hit horizontal line first.                
            // Update tNear;
            tNear = ty;

            if (tNear < tMax)
            {
                // Hit top of this cell. 
                pCell->mFlag = (ECellFlag)(pCell->mFlag | CELL_T_EDGE_HIT);
                // Reverse top edge crossing flag, which is used to determine corner inside or not.
                pCell->mFlag = (ECellFlag)(pCell->mFlag ^ CELL_T_EDGE_PARITY);

                // Get x, y coordinate of intersection point.
                _vb.y = mLatitudes[cy + 1];                    
                _vb.x = va.x + tNear * dir.x;

                // Get next cell.
                ++cy;
                pNextCell = &mCells[cy * mColumn + cx];

                // Hit bottom of the next cell.
                pNextCell->mFlag = (ECellFlag)(pNextCell->mFlag | CELL_B_EDGE_HIT);
                // Reverse bottom edge crossing flag, which is used to determine corner inside or not.
                pNextCell->mFlag = (ECellFlag)(pNextCell->mFlag ^ CELL_B_EDGE_PARITY);
            }
            else
            {
                _vb = vb;                
            }

            // Update ty.
            ty += dty;
        }

        // Add segment (_va, _vb) to this cell.
        SetupEdge(pCell, _va, _vb, dir.x >= 0);
        
        // Go to next.
        pCell = pNextCell;
        _va = _vb;
    };

    return true;
}

/// <description>
/// Construct Edge structure in the cell according to the given edge.
/// </description>
void Grid::SetupEdge(Cell* pCell, const Vector2& p0, const Vector2& p1, bool leftToRight)
{
    float slope, invSlope;
    Vector2 delta = p1 - p0;    
    if (fabs(delta.x) < EPSILON)
    {
        if (delta.y < EPSILON)
        {
            return;
        }
        else
        {
            slope = 0.0f;
            invSlope = INFINITE_VALUE;
        }
    }
    else 
    {
        if (delta.y < EPSILON)
        {
            slope = INFINITE_VALUE;
            invSlope = 0.0f;
        }
        else
        {
            slope = delta.x / delta.y;
            invSlope = 1.0f / slope;
        }
    }
    
    Edge edge;
    edge.mStart = p0;
    edge.mDelta = delta;
    edge.mSlope = slope;
    edge.mInvSlope = invSlope;
    if (leftToRight)
    {
        edge.mMin.x = p0.x;
        edge.mMax.x = p1.x;
    }
    else
    {
        edge.mMin.x = p1.x;
        edge.mMax.x = p0.x;
    }
    edge.mMin.y = p0.y;
    edge.mMax.y = p1.y;

    pCell->mEdgeList.push_back(edge);
    
    // TODO: Assert(edge.mMin.x > mMin.x && edge.mMin.y > mMin.y && edge.mMax.x < mMax.x && edge.mMax.y < mMax.y);
}

/// <description>
/// Determine whether the corners of the cell are inside the polygon or not.
/// </description>
void Grid::CategorizeCellCorner()
{
    Cell* pCell = mCells;
    Cell* pCellAbove = pCell + mColumn;
    for (int i = 1; i < mRow; ++i)
    {        
        bool inside = false;
        for (int j = 0; j < mColumn; ++j)
        {
            if (inside)
            {
                // Left corners are inside.
                pCell->mFlag = (ECellFlag)(pCell->mFlag | CELL_TL_IN);
                pCellAbove->mFlag = (ECellFlag)(pCellAbove->mFlag | CELL_BL_IN);
            }

            if ((pCell->mFlag & CELL_T_EDGE_PARITY) != 0)
                inside = !inside;

            if (inside)
            {
                // Right corners are inside.
                pCell->mFlag = (ECellFlag)(pCell->mFlag | CELL_TR_IN);
                pCellAbove->mFlag = (ECellFlag)(pCellAbove->mFlag | CELL_BR_IN);
            }

            pCell++;
            pCellAbove++;
        }
    }
}

/// <description>
/// Clear all members in Grid except original polygon info.
/// </description>
void Grid::Clear()
{
    mMin.x = 0.0f;
    mMin.y = 0.0f;
    mMax.x = 0.0f;
    mMax.y = 0.0f;

    mRow = 0;
    mColumn = 0;
    mTotalCellCount = 0;

    mCellSize.x = 0.0f;
    mCellSize.y = 0.0f;
    mInvCellSize.x = 0.0f;
    mInvCellSize.y = 0.0f;

    if (mLongitudes != NULL)
    {
        delete [] mLongitudes;
        mLongitudes = NULL;
    }
    if (mLatitudes != NULL)
    {
        delete [] mLatitudes;
        mLatitudes = NULL;
    }

    if (mCells != NULL)
    {
        delete [] mCells;
        mCells = NULL;
    }
}

/// <description>
/// Calculate bounding box of the polygon.
/// TODO: Move to a common library.
/// </description>
void Grid::CalculateBoundingBox(const Vector2* polygon, int count, Vector2& minCorner, Vector2& maxCorner)
{
    minCorner.x = maxCorner.x = polygon[0].x;
    minCorner.y = maxCorner.y = polygon[0].y;

    for (int i = 1; i < count; ++i)
    {
        if (polygon[i].x < minCorner.x)
            minCorner.x = polygon[i].x;
        else if (polygon[i].x > maxCorner.x)
            maxCorner.x = polygon[i].x;

        if (polygon[i].y < minCorner.y)
            minCorner.y = polygon[i].y;
        else if (polygon[i].y > maxCorner.y)
            maxCorner.y = polygon[i].y;
    }
}

/// <description>
/// Test point inside the polygon.
/// </description>
bool Grid::IsPointInPolygon(const Vector2& point)
{
    // Fristly, test whether the point is inside the bounding box.
    if (point.x < mMin.x || point.x > mMax.x ||
        point.y < mMin.y || point.y > mMax.y)
        return false;

    // Find which cell the point locates.
    int cx = int((point.x - mMin.x) * mInvCellSize.x);
    int cy = int((point.y - mMin.y) * mInvCellSize.y);
    Cell* pCell = &mCells[cy * mColumn + cx];


    if (pCell->mEdgeList.size() == 0)
    {
        // If no edge in this cell, check one corner of the cell.
        // If the corner is inside, the cell is inside; or, the cell is outside.
        return ((pCell->mFlag & CELL_BL_IN) != 0);
    }
    else
    {
        // If left edge is not hit.
        if ((pCell->mFlag & CELL_L_EDGE_HIT) == 0)
        {            
            bool inside = ((pCell->mFlag & CELL_BL_IN) != 0);
            for (int i = 0; i < int(pCell->mEdgeList.size()); ++i)
            {
                Edge& edge = pCell->mEdgeList[i];
                if (point.y >= edge.mMin.y && point.y < edge.mMax.y)    // !!!
                {
                    if (point.x > edge.mMax.x)                        
                    {
                        inside = !inside;
                    }
                    else if (point.x > edge.mMin.x)
                    {
                        if ((edge.mStart.x + edge.mSlope * (point.y - edge.mStart.y)) < point.x)
                            inside = !inside;
                    }
                }
            }

            return inside;
        }

        // If right edge is not hit.
        if ((pCell->mFlag & CELL_R_EDGE_HIT) == 0)
        {            
            bool inside = ((pCell->mFlag & CELL_BR_IN) != 0);
            for (int i = 0; i < int(pCell->mEdgeList.size()); ++i)
            {
                Edge& edge = pCell->mEdgeList[i];
                if (point.y >= edge.mMin.y && point.y < edge.mMax.y)    // !!!
                {
                    if (point.x < edge.mMin.x)                        
                    {
                        inside = !inside;
                    }
                    else if (point.x < edge.mMax.x)
                    {
                        if ((edge.mStart.x + edge.mSlope * (point.y - edge.mStart.y)) > point.x)
                            inside = !inside;
                    }
                }
            }

            return inside;
        }

        // If top edge is not hit.
        if ((pCell->mFlag & CELL_T_EDGE_HIT) == 0)
        {            
            bool inside = ((pCell->mFlag & CELL_TL_IN) != 0);
            for (int i = 0; i < int(pCell->mEdgeList.size()); ++i)
            {
                Edge& edge = pCell->mEdgeList[i];
                if (point.x >= edge.mMin.x && point.x < edge.mMax.x)    // !!!
                {
                    if (point.y < edge.mMin.y)  
                    {
                        inside = !inside;
                    }
                    else if (point.y < edge.mMax.y)
                    {
                        if ((edge.mStart.y + edge.mInvSlope * (point.x - edge.mStart.x)) > point.y)
                            inside = !inside;
                    }
                }
            }

            return inside;
        }

        // If bottom edge is not hit.
        if ((pCell->mFlag & CELL_B_EDGE_HIT) == 0)
        {            
            bool inside = ((pCell->mFlag & CELL_BL_IN) != 0);
            for (int i = 0; i < int(pCell->mEdgeList.size()); ++i)
            {
                Edge& edge = pCell->mEdgeList[i];
                if (point.x >= edge.mMin.x && point.x < edge.mMax.x)    // !!!
                {
                    if (point.y > edge.mMax.y)  
                    {
                        inside = !inside;
                    }
                    else if (point.y > edge.mMin.y)
                    {
                        if ((edge.mStart.y + edge.mInvSlope * (point.x - edge.mStart.x)) < point.y)
                            inside = !inside;
                    }
                }
            }

            return inside;
        }

        // Test corner.
        // Connect to Bottom-Left corner.
        Vector2 corner(mLongitudes[cx], mLatitudes[cy]);        

        bool inside = ((pCell->mFlag & CELL_BL_IN) != 0);
        for (int i = 0; i < int(pCell->mEdgeList.size()); ++i)
        {
            Edge& edge = pCell->mEdgeList[i];
            if (point.x >= edge.mMin.x && point.y >= edge.mMin.y)
            {
                if (SegmentSegmentIntersection_Antonio_Direction(point, corner - point, edge.mStart, edge.mDelta))
                    inside = !inside;
            }
        }

        return inside;
    }
}

/// <description>
/// Test whether the line segment intersects the polygon.
/// </description>
bool Grid::IsSegmentIntersectPolygon(const Vector2& p0, const Vector2& p1)
{
    // Clip the line segment by the bounding box of the polygon(rather than the grid).
	Vector2 pa, pb;
	if (!ClipSegment(p0, p1, mMinPolygon, mMaxPolygon, pa, pb))
		return false;

	// Test the line segment to the grid-accelerated polygon.
	// Get normalized direction vector of the line segment.
	Vector2 dir = pb - pa;
	float tMax = sqrt(dir.x * dir.x + dir.y * dir.y);
	if (tMax < EPSILON)
		return false;
	dir /= tMax;
	
	// Find the cell pa locates.
	int cx = int((pa.x - mMin.x) * mInvCellSize.x);
	int cy = int((pa.y - mMin.y) * mInvCellSize.y);

    // TODO: Assert(cx >= 0 && cx < mColumn && cy >= 0 && cy <= mRow);

	int signx, signy;
	float tx, ty;
	float dtx, dty;	
	Vector2 invDir;

	// Get tx, dtx and signx.
	if (fabs(dir.x) < EPSILON)
	{
		tx = INFINITE_VALUE;
		signx = 0;
	}
	else
	{
		invDir.x = 1.0f / dir.x;
		if (dir.x > 0)
		{
			tx = (mLongitudes[cx + 1] - pa.x) * invDir.x;
			dtx = mCellSize.x * invDir.x;
			signx = 1;
		}
		else
		{
			tx = (mLongitudes[cx] - pa.x) * invDir.x;
			dtx = -mCellSize.x * invDir.x;
			signx = -1;
		}
	}

	// Get ty, dty and signy.
	if (fabs(dir.y) < EPSILON)
	{
		ty = INFINITE_VALUE;
		signy = 0;
	}
	else
	{
		invDir.y = 1.0f / dir.y;
		if (dir.y > 0)
		{
			ty = (mLatitudes[cy + 1] - pa.y) * invDir.y;
			dty = mCellSize.y * invDir.y;
			signy = 1;
		}
		else
		{
			ty = (mLatitudes[cy] - pa.y) * invDir.y;
			dty = -mCellSize.y * invDir.y;
			signy = -1;
		}
	}

	// Traveral cells the segment crosses.
	float tNear = 0.0f;
	Vector2 va = pa;
	Vector2 vb;	
	while (tNear < tMax)
	{
        // Get current cell.
        // TODO: Assert(cx >= 0 && cx < mColumn && cy >= 0 && cy <= mRow);
		Cell& currentCell = mCells[cy * mColumn + cx];
		
		// Update tx, cx or ty, cy.
		if (tx <= ty)
		{
			tNear = tx;
			tx += dtx;
			cx += signx;
		}
		else
		{
			tNear = ty;
			ty += dty;
			cy += signy;
		}

		// Get start point for next cell.
		if (tNear < tMax)		
			vb = pa + tNear * dir;		
		else		
			vb = pb;
		
		// Test (va, vb) to the edges in currentCell.		
		if (TestSegmentToEdgesInCell(va, vb, currentCell))
			return true;

		// Move to the next.		
		va = vb;		
	}
	
    
    return false;
}

/// <description>
/// Test line segment against the edges in the given cell.
/// </description>
bool Grid::TestSegmentToEdgesInCell(const Vector2& p0, const Vector2& p1, const Cell& cell)
{
	// Empty cell.
	if (cell.mEdgeList.size() == 0)
		return false;

	// Find the bounding box of (p0, p1).
	Vector2 dir = p1 - p0;
	Vector2 minCorner, maxCorner;
	BoundingBoxOfSegment(p0, p1, dir, minCorner, maxCorner);

	// Iterate edges in the cell and do segment-segment intersection test.
	for (int i = 0; i < int(cell.mEdgeList.size()); ++i)
	{
		const Edge& edge = cell.mEdgeList[i];

		// Box-box intersection test.
		if (BoxBoxIntersect(minCorner, maxCorner, edge.mMin, edge.mMax))
		{
			// Segment-Segment intersection test.
			if (SegmentSegmentIntersection_Antonio_Direction(p0, dir, edge.mStart, edge.mDelta))
				return true;
		}
	}

	return false;
}