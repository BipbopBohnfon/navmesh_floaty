#pragma once

#include "point.h"

namespace NavMesh {

	class Segment
	{
	public:
		Segment() : b(0.0f, 0.0f), e(0.0f, 0.0f) {}
		Segment(Point begin, Point end) : b(begin), e(end) {}

		// Check for intersection with segment |other_begin|-|other_end|.
		//
		// Asymmetric intersection semantics (designed for visibility graph):
		// - Endpoints of |this| segment are NOT counted as intersections.
		// - Endpoints |other_begin| and |other_end| ARE counted as intersections.
		// - Parallel/collinear segments never intersect.
		//
		// This allows edges to touch polygon vertices without being blocked,
		// while preventing edges from crossing through polygon interiors.
		bool Intersects(const Point& other_begin, const Point& other_end) const;

		// Returns the intersection point of this segment's line with another segment's line.
		// Assumes segments are not parallel. Returns midpoint of |this| as fallback if parallel.
		Point GetIntersection(const Point& other_begin, const Point& other_end) const;

		// Equality comparison (direction-independent).
		bool operator==(const Segment& other) const;

		// Segment begin point.
		Point b;

		// Segment end point.
		Point e;
	};

}
