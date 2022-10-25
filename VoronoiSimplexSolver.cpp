#include "Common.h"

#include "VoronoiSimplexSolver.h"

VoronoiSimplexSolver::VoronoiSimplexSolver() : m_numVertices(0)
{
}

void VoronoiSimplexSolver::AddVertex(const Vector2& w)
{
    m_simplexVectorW[m_numVertices] = w;
    m_numVertices++;
}

bool VoronoiSimplexSolver::InSimplex(const Vector2& w)
{
    bool found = false;
    for (int i = 0; i < m_numVertices; ++i)
    {
        if (Vec2LengthSquare(m_simplexVectorW[i] - w) <= VORONOI_DEFAULT_EQUAL_VERTEX_THRESHOLD)
            found = true;
    }

    return found;
}

bool VoronoiSimplexSolver::Closest(Vector2& nearest, const Vector2& p)
{
    switch(m_numVertices)
    {
    case 1:
        {
            nearest = m_simplexVectorW[0];
            return true;
        }        
    case 2:
        {
            return ClosestPtPointLine(p, m_simplexVectorW[0], m_simplexVectorW[1], nearest);            
        }
    case 3:
        {
            return ClosestPtPointTriangle(p, m_simplexVectorW[0], m_simplexVectorW[1], m_simplexVectorW[2], nearest);
        }
    default:
        {
            // Assert(false).
            return false;
        }
    }
}

void VoronoiSimplexSolver::RemoveVertex(int index)
{
    m_numVertices--;
    m_simplexVectorW[index] = m_simplexVectorW[m_numVertices];
}

bool VoronoiSimplexSolver::ClosestPtPointLine(const Vector2& p, const Vector2& a, const Vector2& b, Vector2& nearest)
{
    Vector2 ap = p - a;
    Vector2 ab = b - a;
    float d1 = Vec2Dot(ap, ab);
    float d2 = Vec2Dot(ab);
    
    if (d1 < d2)
    {
        float t = d1 / d2;
        nearest = (1.0f - t) * a + t * b;
    }
    else
    {
        nearest = b;
        RemoveVertex(0);
    }

    return true;
}

bool VoronoiSimplexSolver::ClosestPtPointTriangle(const Vector2& p, const Vector2& a, const Vector2& b, const Vector2& c, Vector2& nearest)
{
    Vector2 ca = a - c;
    Vector2 cb = b - c;
    Vector2 cp = p - c;

    // Check if P in vertex region outside C.
    float d1 = Vec2Dot(cp, ca);
    float d2 = Vec2Dot(cp, cb);
    if (d1 <= 0.0f && d2 <= 0.0f)
    {
        nearest = c;
        RemoveVertex(0);
        RemoveVertex(1);
        return true;
    }

    // Check if P in edge region of CA, if so return projection of P onto CA.
    float d3 = Vec2Dot(ca);
    float d5 = Vec2Dot(ca, cb);
    // vb = (cp x ca) * (cb x ca) = (cp * ca)(cb^2) - (cp * ca)(ca * cb).
    float vb = d2 * d3 - d1 * d5;
    if (d1 > 0.0f && vb <= 0.0f)
    {
        float t = d1 / d3;
        nearest = t * a + (1.0f - t) * c;
        RemoveVertex(1);
        return true;
    }

    // Check if P in edge region of CB, if so return projection of P onto CB.
    float d4 = Vec2Dot(cb);
    // va = (cp x cb) * (cb x ca) = (cp * cb)(ca^2) - (cp * cb)(ca * cb).
    float va = d1 * d4 - d2 * d5;
    if (d2 > 0.0f && va <= 0.0f)
    {
        float t = d2 / d4;
        nearest = t * b + (1.0f - t) * c;
        RemoveVertex(0);
        return true;
    }

    return false;
}