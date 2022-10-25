#include "Common.h"

#include "GJK.h"
#include "Math.h"

void GJK::AddVertex(const Vector2& s)
{
	// TODO: Assert(!IsFullSimplex);

	// Find the room for saving the new vertex.
    m_last = 0;
	m_last_bit = 0x1;
	while (contains(m_bits, m_last_bit))
	{
		++m_last;
		m_last_bit <<= 1;
	}
	m_vertex[m_last] = s;
	//m_length2[m_last] = Vec2Dot(&s, &s);	// TODO: Length2.
	
    // Update m_all_bits to represent the old simplex set combined with the new vertex.
    m_all_bits = m_bits | m_last_bit;

    // Update cache for the new vertex.
	UpdateCache();
	ComputeDet();
}

void GJK::UpdateCache()
{
    // Compute edges (matrix) connecting to the new vertex.
	T_Bits bit = 0x1;
	for (int i = 0; i < FULL_SIMPLEX; ++i, bit <<= 1)
	{
		if (contains(m_bits, bit))
		{
			m_edge[i][m_last] = m_vertex[i] - m_vertex[m_last];
			m_edge[m_last][i] = -m_edge[i][m_last];
		}
	}
}

void GJK::ComputeDet()
{
	m_det[m_last_bit][m_last] = 1.0f;

	if (m_bits != 0x0)
	{
		if (contains(m_bits, 0x1))
		{
            // If the old simplex has A_0 (vertex with index 0), calculate and save:
            // (A_last - A_0) * A_last and (A_0 - A_last) * A0 to determine whether 
            // the origin is within the sweeping volume of the line segment (A_0, A_last) 
            // in its perpendicular direction.
			T_Bits s2 = 0x1 | m_last_bit;
			// TODO: Vec2Dot use reference rather than pointer.
			m_det[s2][0] = Vec2Dot(m_edge[m_last][0], m_vertex[m_last]);
			m_det[s2][m_last] = Vec2Dot(m_edge[0][m_last], m_vertex[0]);
		}

		if (contains(m_bits, 0x2))
		{
            // If the old simplex has A_1 (vertex with index 1), calculate and save:
            // (A_last - A_1) * A_last and (A_1 - A_last) * A_1 to determine whether
            // the origin is within the sweeping volume of the line segment (A_1, A_last)
            // in its perpendicular direction.
			T_Bits s2 = 0x2 | m_last_bit;
			m_det[s2][1] = Vec2Dot(m_edge[m_last][1], m_vertex[m_last]);
			m_det[s2][m_last] = Vec2Dot(m_edge[1][m_last], m_vertex[1]);

			if (contains(m_bits, 0x1))
			{
                // If the old simplex also has A_0,
                // N0, N1 are the nearest points to the origin on the line (A_0, A_last) and (A_1, A_last), so
                // N0' = N0 * |A_0, A_last|^2 = ((A_last - A_0) * A_last) * A_0 + ((A_0 - A_last) * A_0) * A_last, 
                // N1' = N1 * |A_1, A_last|^2 = ((A_last - A_1) * A_last) * A_1 + ((A_1 - A_last) * A_1) * A_last, 
                // save N1' * (A_1 - A_0) to determine whether A_0 is on the opposite side of (A_1, A_last) to the origin, 
                // so the origin is closer to the line segment (A_1, A_last) than (A1, A0).
                // save N0' * (A_0 - A_1) to determine whether A_1 is on the opposite side of (A_0, A_last) to the origin,
                // so the origin is closer to the line segment (A_0, A_last) than (A0, A1).
				T_Bits s3 = 0x1 | s2;
				T_Bits t2 = 0x1 | m_last_bit;
				m_det[s3][0] = Vec2Dot(m_det[s2][1] * m_vertex[1] + m_det[s2][m_last] * m_vertex[m_last], m_edge[1][0]);
				m_det[s3][1] = Vec2Dot(m_det[t2][0] * m_vertex[0] + m_det[t2][m_last] * m_vertex[m_last], m_edge[0][1]);				
			}
		}

		if (contains(m_bits, 0x4))
		{
			T_Bits s2 = 0x4 | m_last_bit;
			m_det[s2][2] = Vec2Dot(m_edge[m_last][2], m_vertex[m_last]); 
			m_det[s2][m_last] = Vec2Dot(m_edge[2][m_last], m_vertex[2]);

            // m_bits cannot simultaneously contains 0x1 and 0x2 when alreacy contains 0x4, since here for 2D application,
            // the old simplex is at most contains 2 vertices, yet a line segment.
			if (contains(m_bits, 0x1))
			{
				T_Bits s3 = 0x1 | s2;
				T_Bits t2 = 0x1 | m_last_bit;
				m_det[s3][0] = Vec2Dot(m_det[s2][2] * m_vertex[2] + m_det[s2][m_last] * m_vertex[m_last], m_edge[2][0]);
				m_det[s3][2] = Vec2Dot(m_det[t2][0] * m_vertex[0] + m_det[t2][m_last] * m_vertex[m_last], m_edge[0][2]);
			}

			if (contains(m_bits, 0x2))
			{
				T_Bits s3 = 0x2 | s2;
				T_Bits t2 = 0x2 | m_last_bit;
				m_det[s3][1] = Vec2Dot(m_det[s2][2] * m_vertex[2] + m_det[s2][m_last] * m_vertex[m_last], m_edge[2][1]);
				m_det[s3][2] = Vec2Dot(m_det[t2][1] * m_vertex[1] + m_det[t2][m_last] * m_vertex[m_last], m_edge[1][2]);
			}
		}
	}
}

bool GJK::IsAffinelyDependent() const
{
    float sum = 0.0f;
    int i;
    T_Bits bit;
    for (i = 0, bit = 0x1; i < FULL_SIMPLEX; ++i, bit <<= 1)
    {
        if (contains(m_all_bits, bit))
        {
            sum += m_det[m_all_bits][i];
        }
    }

    return sum <= 0.0f;
}

bool GJK::Closest(Vector2& d)
{
    // Combine the sub-set of the old simplex with the new vertex to for the new simplex if
    // it contains the nearest point to the origin.
    // Here, for 2D application, the old simplex has at most 2 vertices, and the sub-set has at
    // most one vertex. So, just iterate the vertices one by one.
    T_Bits s;
    for (s = 0x1; s < FULL_SIMPLEX_BITS; s <<= 1)
    {
        if (subseteq(s, m_bits) && IsValid(s | m_last_bit))
        {
            // If succeed to form the new simplex, update m_bits to represent the new simplex.            
            m_bits = s | m_last_bit;

            // Compute the new direction.
            d = ComputeVector(m_bits);
            return true;
        }
    }

    // All fails above, check whether the new vertex is the nearest point to the origin.
    if (IsValid(m_last_bit))
    {
        //m_maxlen2 = m_length2[m_last];        
        m_bits = m_last_bit;        
        d = m_vertex[m_last];
        return true;
    }

    return false;
}

bool GJK::IsValid(T_Bits s) const
{
    int i;
    T_Bits bit;
    for (i = 0, bit = 0x1; i < FULL_SIMPLEX; ++i, bit <<= 1)
    {
        if (contains(m_all_bits, bit))
        {
            if (contains(s, bit))
            {
                // If A_i is within the simplex s, see whether the origin is within
                // the sweeping volume of the line segment (A_i, A_last). See comments in GJK::ComputeDet.
                // If A_i == A_last, m_det[][] is always 1.
                if (m_det[s][i] <= 0.0f)
                {                    
                    return false;
                }
            }
            else 
            {
                // If A_i is not within the simplex s, see whether A_i is on the opposite side 
                // of the line segment (yet simplex s) to the origin.
                if (m_det[s | bit][i] > 0.0f)
                {
                    return false;
                }
            }
        }
    }

    return true;
}

Vector2 GJK::ComputeVector(T_Bits s)
{    
    //m_maxlen2 = 0.0f;

    Vector2 v(0.0f, 0.0f);
    float sum = 0.0f;
    int i;
    T_Bits bit;
    for (i = 0, bit = 0x1; i < FULL_SIMPLEX; ++i, bit <<= 1)
    {
        if (contains(s, bit))
        {           
            //if (m_maxlen2 < m_length2[i])
            //    m_maxlen2 = m_length2[i];

            // Calculate the nearest point to the origin. See comments in GJK::ComputeDet().
            sum += m_det[s][i];
            v += m_vertex[i] * m_det[s][i];
        }
    }

    // TODO: Assert(sum > 0.0f);
    return v / sum;
}
