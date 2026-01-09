#pragma once

#include <vector>
#include <utility>
#include <limits>

#include "point.h"
#include "segment.h"

namespace NavMesh {

	// Axis-aligned bounding box.
	struct AABB {
		float min_x = std::numeric_limits<float>::max();
		float max_x = std::numeric_limits<float>::lowest();
		float min_y = std::numeric_limits<float>::max();
		float max_y = std::numeric_limits<float>::lowest();

		// Check if point is within the bounding box (with epsilon tolerance).
		bool Contains(const Point& p) const {
			return p.x >= min_x - EPSILON && p.x <= max_x + EPSILON &&
			       p.y >= min_y - EPSILON && p.y <= max_y + EPSILON;
		}

		// Check if two bounding boxes overlap.
		bool Intersects(const AABB& other) const {
			return !(max_x < other.min_x || other.max_x < min_x ||
			         max_y < other.min_y || other.max_y < min_y);
		}

		// Check if a segment potentially intersects this bounding box.
		bool IntersectsSegment(const Point& p1, const Point& p2) const {
			// Fast AABB vs segment test using separating axis
			float seg_min_x = std::min(p1.x, p2.x);
			float seg_max_x = std::max(p1.x, p2.x);
			float seg_min_y = std::min(p1.y, p2.y);
			float seg_max_y = std::max(p1.y, p2.y);

			return !(seg_max_x < min_x || seg_min_x > max_x ||
			         seg_max_y < min_y || seg_min_y > max_y);
		}
	};

	class Polygon
	{
	public:
		Polygon();
		Polygon(const Polygon&);
		Polygon(Polygon&&);
		~Polygon();

		Polygon& operator=(Polygon&&);
		Polygon& operator=(const Polygon&);

		// Adds point to the polygon.
		// If it lies inside the polygon or on the side,
		// it will be ignored.
		void AddPoint(const Point& a);
		void AddPoint(float x, float y);

		// Checks if |a| is strictly inside the polygon.
		// Returns false if |a| coincides with some vertex
		// or lies on the side.
		bool IsInside(const Point& a) const;

		// Returns the axis-aligned bounding box of the polygon.
		// Computed lazily and cached.
		const AABB& GetBoundingBox() const;

		// Number of points in the polygon.
		int Size() const;

		// Checks if |s| intersects any side of this polygon.
		// |tangents| are precomputed tangents from |s.b| to the polygon.
		//
		// Note: neither endpoint of |s| could be inside the polygon.
		// In that case this can return wrong values!
		//
		// Only interior of |s| and polygon is counted as intersections.
		// So, |s| can touch the polygon with an end, touch a side or
		// cross a vertex without intersection.
		// This is to allow paths to pass between touching polygons.
		bool Intersects(const Segment& s, const std::pair<int, int>& tangents) const;

		// Checks if |s| intersects any side of this polygon.
		// Does not require pre-computed tangents (O(k) where k is vertex count).
		// Uses bounding box for early rejection.
		bool IntersectsNaive(const Segment& s) const;

		// Removes all points.
		void Clear();

		// Returns corresponding point.
		// No boundary checks are made.
		const Point& operator[](size_t i) const;

		// Inflates the polygon such that resulting boundary
		// is at least |r| units away from the polygon.
		//
		// Constructs Minkowski sum of the polygon and 2*r x 2*r square centered at 0.
		// Increases number of points by at most 4.
		Polygon Inflate(float r) const;

		// Returns ids of two points which are endpoints for
		// two tangents (in different directions).
		// First id is for the left-most point visible from |a|.
		// Second id is for the right-most point.
		//
		// "Closest" points are returned, so if |a| is lying on the
		// continuation of some side, the closest of two vertices
		// would be returned.
		//
		// Returns {-1, -1} if |a| is inside the polygon.
		// Returns {i, i}, if |a| is the i-th vertex of the polygon.
		// If |a| is on the side, returns two consecutive ids.
		std::pair<int, int> GetTangentIds(const Point& a) const;

		// Checks if the segment from |a| to i-th point is tangent to
		// this polygon. I.e. i-th point is the leftmost or the rightmost,
		// if seen from |a|.
		bool IsTangent(int i, const Point& a) const;

	private:
		friend class PathFinder;

		// Minimum polygon size to use O(log n) tangent algorithm instead of O(n) naive.
		static constexpr int kMinPointsForLogTangentsAlgo = 4;

		// Maximum iterations for inflation loop (safety bound).
		static constexpr int kMaxInflationIterations = 1000;

		// Number of sides in the inflation square (Minkowski sum).
		static constexpr int kInflationSquareSides = 4;

		// Line equation coefficients: (a, b) for ax + by + c = 0.
		// b is always positive for consistent orientation.
		using LineCoefficients = std::pair<float, float>;

		// Full line equation: ((a, b), c) for ax + by + c = 0.
		using LineEquation = std::pair<LineCoefficients, float>;

		// Points of the polygon.
		// Always in counter-clockwise order.
		std::vector<Point> points_;

		// Cached axis-aligned bounding box.
		mutable AABB bbox_;
		mutable bool bbox_valid_ = false;

		// Computes and caches the bounding box.
		void ComputeBoundingBox() const;

		// Invalidates cached data when polygon changes.
		void InvalidateCaches();

		// Precalculates data-structures for O(log |p|) IsInside
		// algorithm.
		// If called, would require O(|p|) additional memory.
		void PrepareForFastInsideQueries() const;

		void OrderCounterClockwiseAndRemoveCollinearPoints();

		// All points x coordinates sorted. Used for fast IsInside algorithm.
		mutable std::vector<float> xs_;

		// Line equations for top and bottom boundaries at each vertical segment.
		// i-th entry corresponds to lines between xs_[i] and xs_[i+1].
		mutable std::vector<LineEquation> top_lines_;
		mutable std::vector<LineEquation> bottom_lines_;

		std::pair<int, int> GetTangentIdsNaive(const Point& a) const;
		std::pair<int, int> GetTangentIdsLogarithmic(const Point& a) const;
	};

}
