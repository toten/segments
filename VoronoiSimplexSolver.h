#ifndef _VORONOI_SIMPLEX_SOLVER_
#define _VORONOI_SIMPLEX_SOLVER_

#include "Math.h"

#define VORONOI_DEFAULT_EQUAL_VERTEX_THRESHOLD 0.0001f
#define VORONOI_SIMPLEX_MAX_VERTS 3

class VoronoiSimplexSolver
{
protected:

    int m_numVertices;
    Vector2	m_simplexVectorW[VORONOI_SIMPLEX_MAX_VERTS];    
    
    void RemoveVertex(int index);
    bool ClosestPtPointLine(const Vector2& p, const Vector2& a, const Vector2& b, Vector2& nearest);
    bool ClosestPtPointTriangle(const Vector2& p, const Vector2& a, const Vector2& b, const Vector2& c, Vector2& nearest);

public:
    VoronoiSimplexSolver();

    bool InSimplex(const Vector2& w);
    void AddVertex(const Vector2& w);
    bool Closest(Vector2& nearest, const Vector2& p = Vector2(0.0f, 0.0f));
};

#endif