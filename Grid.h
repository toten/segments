#ifndef _GRID_H_
#define _GRID_H_

#include "Math.h"
#include "Array.h"

/// <description>
/// Flag for cell.
/// </description>
enum ECellFlag
{
    None                        = 0x0,
     
    CELL_BL_IN                  = 0x1,
    CELL_BR_IN                  = 0x1 << 1,
    CELL_TL_IN                  = 0x1 << 2,
    CELL_TR_IN                  = 0x1 << 3,

    CELL_L_EDGE_HIT             = 0x1 << 4,
    CELL_R_EDGE_HIT             = 0x1 << 5,
    CELL_B_EDGE_HIT             = 0x1 << 6,
    CELL_T_EDGE_HIT             = 0x1 << 7,

    CELL_B_EDGE_PARITY          = 0x1 << 8,
    CELL_T_EDGE_PARITY          = 0x1 << 9,

    //CELL_AIM_L                  = 0x1 << 10,
    //CELL_AIM_B                  = 0x1 << 11,
    //CELL_AIM_R                  = 0x1 << 12,
    //CELL_AIM_T                  = 0x1 << 13,
    //CELL_AIM_C                  = 0x1 << 14
};

/// <description>
/// Edge structure.
/// </description>
struct Edge
{
    Vector2                     mStart;

    Vector2                     mDelta;

    Vector2                     mMin;

    Vector2                     mMax;

    float                       mSlope;

    float                       mInvSlope;

    /// <description>
    /// Constructor.
    /// </description>
    Edge()
        : mStart(0.0f, 0.0f)
        , mDelta(0.0f, 0.0f)
        , mSlope(0.0f)
        , mInvSlope(0.0f) 
    {}

    /// <description>
    /// Destructor.
    /// </description>
    ~Edge() {}
};

/// <description>
/// Cell structure.
/// </description>
struct Cell
{
    // Cell flag.
    ECellFlag                   mFlag;
    
    // Edge list.
    Array<Edge>                  mEdgeList;

    /// <description>
    /// Constructor.
    /// </description>
    Cell() : mFlag(None) {}

    /// <description>
    /// Destructor.
    /// </description>
    ~Cell() {}
};

/// <description>
/// Grid acceleration structure.
/// </description>
class Grid
{
protected:
    // Infomation of original polygon.
    int                         mTotalEdgeCount;
    const Vector2*              mVertices;

	// Bounding box of the polygon.
	Vector2						mMinPolygon;
	Vector2						mMaxPolygon;

    // Bounding box of the grid.
    Vector2                     mMin;
    Vector2                     mMax;

    // Row and column number. TotalCount = mRow * mColumn.    
    int                         mRow;
    int                         mColumn;
    int                         mTotalCellCount;

    // Size of the cell and the inverse.
    Vector2                     mCellSize;
    Vector2                     mInvCellSize;

    // Coordinates for longtitudes and latitudes of the grid.
    float*                      mLongitudes;
    float*                      mLatitudes;

    // Cells.
    Cell*                       mCells;

protected:
    // Bounding box expansion factor.
    static float                EXPAND_INCREMENT;

    // Maximum edge number on average in one cell.
    static int                  MAXIMUM_EDGE_NUMBER;

    // Maximum times of re-create grid.
    static int                  MAXIMUM_RECREATE;

public:
    /// <description>
    /// Create a grid acceleration structure from the given polygon.
    /// Return NULL if fail to create the structure.
    /// </description>
    static Grid* CreateGrid(const Vector2* polygon, int count);

    /// <description>
    /// Destroy the grid structure.
    /// </description>
    static void DestroyGrid(Grid* pGrid);

public:
    /// <description>
    /// Test point inside the polygon.
    /// </description>
    bool IsPointInPolygon(const Vector2& point);

    /// <description>
    /// Test whether the line segment intersects the polygon.
    /// </description>
    bool IsSegmentIntersectPolygon(const Vector2& p0, const Vector2& p1);

protected:
    /// <description>
    /// Constructor. Called internally.
    /// </description>
    Grid(const Vector2* polygon, int count);

    /// <description>
    /// Destructor. Called internally.
    /// </description>
    virtual ~Grid();

protected:
    /// <description>
    /// Initialize grid structure.
    /// </description>
    bool Initialize();

    /// <description>
    /// Setup grid.
    /// </description>
    bool Setup();    

    /// <description>
    /// Divide the bounding box of the polygon into mColumn * mRow grid.
    /// </description>
    void SubDivide();

    /// <description>
    /// Loop through edges and insert into grid structure.
    /// </description>
    bool CategorizeEdge();

    /// <description>
    /// Deal with the given edge and intert it into grid structrue.
    /// This method assumes that va.y <= vb.y.
    /// </description>
    bool CategorizeEdge(const Vector2& va, const Vector2& vb);

    /// <description>
    /// Construct Edge structure in the cell according to the given edge,
    /// and add to the cell.
    /// </description>
    void SetupEdge(Cell* pCell, const Vector2& p0, const Vector2& p1, bool leftToRight);

    /// <description>
    /// Determine whether the corners of the cell are inside the polygon or not.
    /// </description>
    void CategorizeCellCorner();

    /// <description>
    /// Clear grid.
    /// </description>
    void Clear();

	/// <description>
	/// Test line segment against the edges in the given cell.
	/// </description>
	bool TestSegmentToEdgesInCell(const Vector2& p0, const Vector2& p1, const Cell& cell);

protected:
    /// <description>
    /// Calculate bounding box of the polygon.
    /// TODO: Move to a common library.
    /// </description>
    static void CalculateBoundingBox(const Vector2* polygon, int count, Vector2& minCorner, Vector2& maxCorner);
};

#endif