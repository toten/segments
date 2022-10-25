#ifndef _GJK_H_
#define _GJK_H_

#include "Math.h"

class GJK
{
private:
	typedef unsigned int T_Bits;
	inline static bool subseteq(T_Bits a, T_Bits b) { return (a & b) == a; }
	inline static bool contains(T_Bits a, T_Bits b) { return (a & b) != 0x0; }

public:
	GJK() : m_bits(0x0), m_all_bits(0x0) {}

	bool IsFullSimplex() const { return m_bits == FULL_SIMPLEX_BITS; }

	void AddVertex(const Vector2& s);

	bool IsAffinelyDependent() const;

	bool Closest(Vector2& d);
	
private:
	const static int FULL_SIMPLEX = 3;
	const static int FULL_SIMPLEX_BITS = 8;

	float		m_det[FULL_SIMPLEX_BITS][FULL_SIMPLEX];
	

	Vector2		m_p[FULL_SIMPLEX];
	Vector2		m_q[FULL_SIMPLEX];
	Vector2		m_vertex[FULL_SIMPLEX];                 // The vertices of the simplex.
	float		m_length2[FULL_SIMPLEX];                // The square length of the vertices of the simplex.
    Vector2		m_edge[FULL_SIMPLEX][FULL_SIMPLEX];     // The edges of the simplex (including two directions)

	float		m_maxlen2;

	int			m_last;
	T_Bits		m_last_bit;
	T_Bits		m_bits;         // The bits represent the old simplex set.
	T_Bits		m_all_bits;     // The combination of the old simplex set and the newly added vertex.

private:
	void UpdateCache();

	void ComputeDet();

	bool IsValid(T_Bits s) const;

	Vector2 ComputeVector(T_Bits s);
};

#endif