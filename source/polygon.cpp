#include "polygon.h"

#include <algorithm>
#include <limits>

namespace NavMesh {

	Polygon::Polygon() = default;

	Polygon::Polygon(const Polygon&) = default;


	Polygon::Polygon(Polygon&& other) :
		points_(std::move(other.points_)),
		xs_(std::move(other.xs_)),
		top_lines_(std::move(other.top_lines_)),
		bottom_lines_(std::move(other.bottom_lines_))
	{ }

	Polygon::~Polygon() = default;

	Polygon& Polygon::operator=(Polygon&& other)
	{
		points_ = std::move(other.points_);
		xs_ = std::move(other.xs_);
		top_lines_ = std::move(other.top_lines_);
		bottom_lines_ = std::move(other.bottom_lines_);
		return *this;
	}

	Polygon& Polygon::operator=(const Polygon& other) {
		points_ = other.points_;
		xs_ = other.xs_;
		top_lines_ = other.top_lines_;
		bottom_lines_ = other.bottom_lines_;
		return *this;
	}

	// TODO: Add fast path for when the new point is already provided
	// in the correct order. Between last and the first.
	// Make |points| private.
	void Polygon::AddPoint(const Point& a)
	{
		if (points_.size() <= 2) {
			points_.push_back(a);
			xs_.clear();
			OrderCounterClockwiseAndRemoveCollinearPoints();
			return;
		}

		// Check if this point fits nicely after the last one.
		// Fast path for already ordered points being added.
		Point prev_side = points_.back() - points_[points_.size() - 2];
		Point new_side1 = a - points_.back();
		Point new_side2 = points_[0] - a;
		Point next_side = points_[1] - points_[0];
		if (FloatSign(prev_side ^ new_side1) > 0 &&
			FloatSign(new_side1 ^ new_side2) > 0 &&
			FloatSign(new_side2 ^ next_side) > 0) {
			points_.push_back(a);
			xs_.clear();
			return;
		}

		// Remove points, which will be inside the convex hull.
		// Find tangents from the newly added point and make them
		// two new sides, removing all points inside.

		auto ids = GetTangentIds(a);
		int i1 = ids.first;
		int i2 = ids.second;

		// No tangents. May happen if |a| is inside or on the boundary.
		// Only one tangent - this means that |a| coincides with some point.
		if (i1 < 0 || i2 < 0 || i1 == i2)
			return;

		// Check if we are on the side.
		if (i2 == (i1 + 1) % (int)points_.size()) {
			Point v1 = points_[i1] - a;
			Point v2 = points_[i2] - a;
			if (FloatZero(v1 ^ v2) && (v1 * v2) <= EPSILON) return;
		}

		// Erase points from i1+1 to i2-1 inclusive and wrapping.
		int insert_to = 0;
		if (i2 > i1) {
			// Shift points after i2 to immediately after i1.
			int shift = i2 - i1 - 1;
			for (size_t i = i1 + 1; i < points_.size() - shift; ++i) {
				points_[i] = points_[i + shift];
			}
			points_.resize(points_.size() - shift);
			insert_to = i1;
		}
		else {
			// Remove points before i2 or after i1.
			// Shift all remaining points by i2 positions left.
			int total = i1 - i2 + 1;
			for (int i = 0; i < total; ++i) {
				points_[i] = points_[i + i2];
			}
			points_.resize(total);
			insert_to = total - 1;
		}
		points_.resize(points_.size() + 1);
		// Shift points right by 1 to make a place for |a|.
		for (size_t i = points_.size() - 1; i > (size_t)(insert_to + 1); --i) {
			points_[i] = points_[i - 1];
		}
		points_[insert_to + 1] = a;

		xs_.clear();
	}

	void Polygon::AddPoint(float x, float y)
	{
		AddPoint(Point(x, y));
	}

	bool Polygon::IsInside(const Point& a) const
	{
		if (xs_.empty()) {
			PrepareForFastInsideQueries();
		}

		// Find, which vertical column the point belongs to.
		// It's such i, that |xs_[i] <= a.x && xs_[i+1] > a.x|.

		// Find first |xs_[i]|, larger than |a.x|.
		auto it = std::upper_bound(xs_.begin(), xs_.end(), a.x);

		// Points lying to the left of the first column, or to
		// the right of the last column can't be inside the polygon.
		if (a.x <= xs_.front() + EPSILON || a.x >= xs_.back() - EPSILON) {
			return false;
		}

		// Previous entry is smaller or equal than |a.x|.
		size_t i = it - xs_.begin() - 1;
		const auto& top = top_lines_[i];
		const auto& bottom = bottom_lines_[i];
		// Plug a.x into y=k*x+b of the lines to get at
		// that y are the boundaries for the given x.
		// Check that a.y lies strictly between them.
		float top_val = top.first.first * a.x + top.first.second * a.y + top.second;
		float bottom_val = bottom.first.first * a.x + bottom.first.second * a.y + bottom.second;
		return (top_val < -EPSILON) && (bottom_val > EPSILON);
	}

	int Polygon::Size() const
	{
		return static_cast<int>(points_.size());
	}

	bool Polygon::Intersects(const Segment& s, const std::pair<int, int>& tangents) const
	{
		// No tangents means that s.b lies inside of the polygon.
		// But it's already guaranteed by PathFinder that no endpoint of |s|
		// is inside, so this check can be skipped:
		//   if (tangents.first < 0 || tangents.second < 0) {
		//	   return false;
		//   }

		const Point v = s.e - s.b;
		Point l;
		Point r;
		Point le;
		Point lr;
		float chord_dir;
		float l_dir;
		float r_dir;

		if (tangents.first == tangents.second) {
			int n = (int)points_.size();
			// One end is a vertex of the polygon.
			// Intersection only if the vector is
			// strictly between two sides.
			l = points_[(tangents.first - 1 + n) % n] - s.b;
			r = points_[(tangents.second + 1) % n] - s.b;
			return FloatSign(l ^ v) < 0 && FloatSign(v ^ r) < 0;
		}

		l = points_[tangents.first] - s.b;
		r = points_[tangents.second] - s.b;
		le = s.e - points_[tangents.first];
		lr = points_[tangents.second] - points_[tangents.first];
		chord_dir = lr ^ le;
		l_dir = l ^ v;
		r_dir = v ^ r;

		// Endpoint lies beyond the chord.
		// Intersection if |v| between |l| and |r| or even touches them.
		return FloatSign(chord_dir) > 0 && FloatSign(l_dir) < 0 && FloatSign(r_dir) < 0;
	}

	void Polygon::Clear()
	{
		points_.clear();
		xs_.clear();
	}

	const Point& Polygon::operator[](size_t i) const
	{
		return points_[i];
	}

	Polygon Polygon::Inflate(float r) const
	{
		Polygon res;

		if (FloatZero(r)) {
			res = *this;
			return res;
		}

		if (points_.empty()) return res;

		// Handle degenerate case: single point becomes a square
		if (points_.size() == 1) {
			res.points_.push_back(Point(points_[0].x - r, points_[0].y - r));
			res.points_.push_back(Point(points_[0].x + r, points_[0].y - r));
			res.points_.push_back(Point(points_[0].x + r, points_[0].y + r));
			res.points_.push_back(Point(points_[0].x - r, points_[0].y + r));
			return res;
		}

		// Sides of a 2*r x 2*r square.
		Point inflation_sides[4] = { {2*r, 0.0f}, {0.0f, 2*r}, {-2*r, 0.0f}, {0.0f, -2*r} };

		// Reserve capacity: original polygon points + up to 4 square corners
		res.points_.reserve(points_.size() + 4);

		// Find leftmost bottom corner.
		int start = 0;
		for (size_t i = 0; i < points_.size(); ++i) {
			if (points_[i] < points_[start]) start = (int)i;
		}

		// Apply either sides of the polygon or of the square: whichever comes first in CCW order.
		int cur_inflation = 0;
		int cur_point_id = start;
		Point cur_point = points_[start] + Point(-r, -r);

		// Safety counter: theoretical max is n + 4, use n * 2 + 8 for safety
		int max_iterations = (int)points_.size() * 2 + 8;
		int iterations = 0;

		do {
			res.points_.push_back(cur_point);
			int next = (cur_point_id + 1) % (int)points_.size();
			Point side = points_[next] - points_[cur_point_id];
			float dir = side ^ inflation_sides[cur_inflation];

			if (FloatSign(dir) > 0) {
				cur_point = cur_point + side;
				cur_point_id = next;
			}
			else if (FloatSign(dir) < 0) {
				cur_point = cur_point + inflation_sides[cur_inflation];
				cur_inflation = (cur_inflation + 1) % 4;
			}
			else {
				// Collinear case: advance both to avoid getting stuck
				cur_point = cur_point + side + inflation_sides[cur_inflation];
				cur_inflation = (cur_inflation + 1) % 4;
				cur_point_id = next;
			}

			iterations++;
			if (iterations > max_iterations) break;

		} while (cur_inflation != 0 || cur_point_id != start);

		return res;
	}

	void Polygon::PrepareForFastInsideQueries() const
	{
		xs_.clear();
		top_lines_.clear();
		bottom_lines_.clear();
		xs_.reserve(points_.size());
		for (const auto& point : points_) {
			xs_.push_back(point.x);
		}
		std::sort(xs_.begin(), xs_.end());
		xs_.erase(std::unique(xs_.begin(), xs_.end(), [](float a, float b) {
			return FloatEqual(a, b);
		}), xs_.end());

		top_lines_.resize(xs_.size());
		bottom_lines_.resize(xs_.size());

		// |cur_x| should always point to the element in |xs_|,
		// which is equal to |points[i].x|.
		size_t cur_x = 0;
		// Naively find cur_x for |points[0].x|.
		for (cur_x = 0; cur_x < xs_.size(); ++cur_x) {
			if (FloatEqual(xs_[cur_x], points_[0].x)) break;
		}

		for (size_t i = 0; i < points_.size(); ++i) {
			const Point& next = points_[(i + 1) % points_.size()];
			if (next.x > points_[i].x + EPSILON) {
				// Side goes left to right: It's a bottom side.

				// Find equation for current side: a*x+b*y+c = 0.
				// b should be positive.
				float a = points_[i].y - next.y;
				float b = next.x - points_[i].x;
				float c = -a * next.x - b * next.y;
				auto line = std::make_pair(std::make_pair(a, b), c);

				// Current line is the bottom for all vertical segments,
				// which start strictly before the right end of this side.
				for (; cur_x < xs_.size() && xs_[cur_x] < next.x - EPSILON; ++cur_x) {
					bottom_lines_[cur_x] = line;
				}
			}
			else if (next.x < points_[i].x - EPSILON) {
				// Side goes right to left: It's a top side.

				// Find equation for current side: a*x+b*y+c = 0.
				// b should be positive.
				float a = next.y - points_[i].y;
				float b = points_[i].x - next.x;
				float c = -a * next.x - b * next.y;
				auto line = std::make_pair(std::make_pair(a, b), c);

				// Current line is the top for all vertical segments,
				// which start strictly before current x, and after or at
				// the next x.
				if (cur_x > 0) {
					for (cur_x = cur_x - 1; cur_x > 0 && xs_[cur_x] > next.x + EPSILON; --cur_x) {
						top_lines_[cur_x] = line;
					}
					if (xs_[cur_x] > next.x + EPSILON && cur_x == 0) {
						// Handle cur_x == 0 case
					} else {
						top_lines_[cur_x] = line;
					}
				}
			}
			else {
				// Vertical line. Skip it.
			}
		}

	}

	void Polygon::OrderCounterClockwiseAndRemoveCollinearPoints()
	{
		if (Size() < 3) return;
		if (FloatSign((points_[1] - points_[0]) ^ (points_[2] - points_[1])) < 0) {
			std::reverse(points_.begin(), points_.end());
		}

		// Remove points on sides or concave direction.
		int n = (int)points_.size();
		std::vector<bool> bad_point(n);
		for (int i = 0; i < n; ++i) {
			Point v1 = points_[i] - points_[(i - 1 + n) % n];
			Point v2 = points_[(i + 1) % n] - points_[i];
			float dir = v1 ^ v2;
			if (FloatSign(dir) < 0) {
				//  Concave point.
				bad_point[i] = true;
			}
			else if (FloatZero(dir) && (v1 * v2) > EPSILON) {
				// Point on side.
				bad_point[i] = true;
			}
		}
		int j = 0;
		for (int i = 0; i < n; ++i) {
			if (!bad_point[i]) {
				points_[j++] = points_[i];
			}
		}
		points_.resize(j);
	}

	std::pair<int, int> Polygon::GetTangentIdsNaive(const Point& a) const
	{
		std::pair<int, int> res = { -1, -1 };
		const int n = (int)points_.size();
		if (n == 1) {
			return { 0, 0 };
		}
		if (n == 2) {
			Point to_0 = points_[0] - a;
			Point to_1 = points_[1] - a;
			float dir = to_0 ^ to_1;
			if (FloatSign(dir) < 0) {
				return { 0, 1 };
			}
			else if (FloatSign(dir) > 0) {
				return { 1, 0 };
			}
			else {
				// dir == 0, collinear points.
				if ((to_0 * to_1) >= -EPSILON) {
					// |a| lies on the line outside of the segment or on the vertex.
					return to_0.Len2() < to_1.Len2() ? std::make_pair(0, 0) : std::make_pair(1, 1);
				}
				else {
					// |a| lies inside the segment.
					return { 0, 1 };
				}
			}
		}
		// Use naive O(|n|) bruteforce algorithm.
		float dir_prev;
		// Precompute for the first vertex.
		Point vprev = points_[0] - points_[n - 1];
		Point to_vprev = points_[n - 1] - a;
		dir_prev = to_vprev ^ vprev;
		for (int i = 0; i < n; ++i) {
			if (points_[i] == a) {
				// Return closest points as the tangent.
				return { i, i };
			}
			Point to_vi = points_[i] - a;
			Point vi = points_[(i + 1) % n] - points_[i];
			float dir_i = to_vi ^ vi;

			// Positive values are for "dark side" sides.
			// Negative values are for front side.
			// Zeros can separate them on one or both sides.
			// "+" are always present, but there may be no "-" (if |a| is some vertex).

			// First tangent is the  "0/-" vertex, preceded by "+" vertex
			if (FloatSign(dir_i) <= 0 && FloatSign(dir_prev) > 0) {
				// This is tangent going "forward" in along the polygon.
				res.first = i;
			}

			// Second tangent is the "+" vertex, preceded by "0/-" vertex.
			if (FloatSign(dir_i) > 0 && FloatSign(dir_prev) <= 0) {
				// This is tangent going backward along the polygon.
				res.second = i;
			}
			dir_prev = dir_i;
		}

		if (res.first < 0 || res.second < 0) return res;

		// Special case if |a| is on the continuation of a side.
		// The solutions above would find the farthest vertex as a tangent end.
		// For path finding algorithms on the graph, however, they must be the closest.
		int next = (res.first + 1) % n;
		Point v = points_[next] - points_[res.first];
		Point to_v = points_[res.first] - a;
		Point v_next = points_[(next + 1) % n] - points_[next];
		Point to_next = points_[next] - a;

		// Should choose the next vertex, if found is "0" vertex and the next is "-/0".
		if (FloatZero(to_v ^ v) && FloatSign(to_next ^ v_next) <= 0) {
			res.first = next;
		}

		int prev = (res.second + n - 1) % n;
		Point v_prev = points_[res.second] - points_[prev];
		Point to_prev = points_[prev] - a;

		// Should choose the previous vertex, if that previous point is "0" node,
		// but not the same vertex, which the first tangent is pointing at, unless |a| is equal to it.
		if ((res.first != prev || a == points_[prev]) && FloatZero(to_prev ^ v_prev)) {
			res.second = prev;
		}

		return res;
	}


	std::pair<int, int> Polygon::GetTangentIdsLogarithmic(const Point& a) const
	{
		std::pair<int, int> res = { -1, -1 };
		const int n = (int)points_.size();

		// Value of the point i is (point[i] - a) ^ (point[i+1] - point[i]).
		// It is positive, if the side rotates counter-clockwise from the segment a-point[i].
		// For back or "dark side" sides it is always positive. It can be 0 for one or two nodes,
		// If |a| lies on the continuation of the side/sides/vertex.
		// It can be negative for the front sides.
		// Goal is to find boundaries between "+" and "0/-" continuous regions.
		// The issue is that points are wrapping around, and we can have "+++--+++" or "---++++---" cases.
		// The trick is to look at the value of the beginning and middle in the region. If they are different,
		// we already can figure out which half to discard.
		// If the values are the same we can figure out if we are still in the same region, or jumped over
		// a different region by looking at relative rotation between points[m]-a and points[l]-a.


		// 1. Search for the left tangent: a "0/-" node, preceded by "+" node.
		Point vl = points_[1] - points_[0];
		Point to_l = points_[0] - a;
		Point v_prev = points_[0] - points_[n - 1];
		Point to_prev = points_[n - 1] - a;
		float pointing_l = to_l ^ vl;
		float pointing_prev = to_prev ^ v_prev;
		int l = 0;
		int r = n - 1;
		// Explicit check if the first is already the tangent.
		if (FloatSign(pointing_prev) > 0 && FloatSign(pointing_l) <= 0) {
			res.first = 0;
			// Check if we could move it forward:
			// if 0-th node is a "0" node and the next is "-/0" node.
			// If both 0-th and 1-st nodes are "0" nodes, then |a == points[1]|.
			if (FloatZero(pointing_l)) {
				Point v_next = points_[2] - points_[1];
				Point to_next = points_[1] - a;
				float pointing_next = to_next ^ v_next;
				if (FloatSign(pointing_next) < 0) {
					res.first = 1;
				}
				else if (FloatZero(pointing_next)) {
					return { 1, 1 };
				}
			}
		}
		else {
			// Loop invariant: switch from "+" to "-/0" guaranteed to happen
			// from between l and r.
			while (l < r - 1) {
				int m = (r + l + 1) / 2;
				// No need to do (m+1)%n, since r-l > 1, it's guaranteed that
				// l < m < r. Hence m+1 is still a valid index.
				Point vm = points_[m + 1] - points_[m];
				Point to_m = points_[m] - a;
				float pointing_m = to_m ^ vm;
				float lm_dir = to_l ^ to_m;

				// The switch happened between l and m iff:
				// - l-th  is "+" vertex and m-th is "-/0" vertex.
				//   I.e. |pointing_l > 0 && pointing_m <= 0|
				// - both are "+" vertices and |m| is strictly to the right of |l|.
				//   I.e. |pointing_l > 0 && pointing_m > 0 && lm_dir < 0|
				// - both are "0/-" and |m| is non-strictly to the left of "l"
				//   I.e. |pointing_l <= 0 && pointing_m <= 0 && lm_dir >= 0|
				//
				// Below code is simplification of the 3 conditions above.
				if ((FloatSign(pointing_m) <= 0 || FloatSign(lm_dir) < 0) &&
					(FloatSign(pointing_l) > 0 || FloatSign(lm_dir) >= 0)) {
					// Overshoot.
					// |m| is the desired point or something to the right of it.
					r = m;
				}
				else {
					l = m;
					to_l = to_m;
					pointing_l = pointing_m;
				}
			}
			// In case the point is inside
			// bin search will do something random.
			// Check that there's indeed switch from "+" to "0/-"
			// between |l| and |r|

			int next = (r + 1) % n;
			Point to_r = points_[r] - a;
			Point vr = points_[next] - points_[r];
			float pointing_r = to_r ^ vr;
			if (FloatSign(pointing_r) <= 0 && FloatSign(pointing_l) > 0) {
				res.first = r;
				// Check if we could move the point forward.
				Point v_next = points_[(next + 1) % n] - points_[next];
				Point to_next = points_[next] - a;
				float pointing_next = to_next ^ v_next;

				// Should choose the next vertex, if found is "0" vertex and the next is "-/0".
				// If the both r and next are 0, then |a = points[r + 1]|
				if (FloatZero(pointing_r)) {
					if (FloatSign(pointing_next) < 0) {
						res.first = next;
					}
					else if (FloatZero(pointing_next)) {
						return { next, next };
					}
				}
			}
			else return { -1, -1 };
		}

		// 2. Search for the right tangent: a "+" node, preceded by "-/0" node.
		vl = points_[1] - points_[0];
		to_l = points_[0] - a;
		v_prev = points_[0] - points_[n - 1];
		to_prev = points_[n - 1] - a;
		pointing_l = to_l ^ vl;
		pointing_prev = to_prev ^ v_prev;
		l = 0;
		r = n - 1;
		// Explicit check if the first is already the tangent.
		if (FloatSign(pointing_prev) <= 0 && FloatSign(pointing_l) > 0) {
			res.second = 0;
			// Check if we could move it forward.
			// Can do it iff the previous node is "0" vertex,
			// but not the same vertex, which the first tangent is pointing at, unless |a| is equal to it.
			// But the equality case is already processed after the first binary search.
			if (res.first != n - 1 && FloatZero(pointing_prev)) {
				res.second = n - 1;
			}
		}
		else {
			// Loop invariant: switch from "-/0" to "+" guaranteed to happen
			// from between l and r.
			while (l < r - 1) {
				int m = (r + l + 1) / 2;
				// No need to do (m+1)%n, since r-l > 1, it's guaranteed that
				// l < m < r. Hence m+1 is still a valid index.
				Point vm = points_[m + 1] - points_[m];
				Point to_m = points_[m] - a;
				float pointing_m = to_m ^ vm;
				float lm_dir = to_l ^ to_m;

				// The switch happened between l and m iff:
				// - l-th  is "0/-" vertex and m-th is "+" vertex.
				//   I.e. |pointing_l <= 0 && pointing_m > 0|
				// - both are "+" vertices and |m| is non-strictly to the right of |l|.
				//   I.e. |pointing_l > 0 && pointing_m > 0 && lm_dir <= 0|
				// - both are "0/-" and |m| is strictly to the left of "l"
				//   I.e. |pointing_l <= 0 && pointing_m <= 0 && lm_dir > 0|
				//
				// Below code is the simplification of the 3 conditions above.
				if ((FloatSign(pointing_l) <= 0 || FloatSign(lm_dir) <= 0) &&
					(FloatSign(pointing_m) > 0 || FloatSign(lm_dir) > 0)) {
					// Overshoot.
					// |m| is the desired point or something to the right of it.
					r = m;
				}
				else {
					l = m;
					to_l = to_m;
					pointing_l = pointing_m;
				}
			}
			// In case the point is inside
			// bin search will do something random.
			// Check that there's indeed switch from "0/-" to "+"
			// between |l| and |r|
			Point to_r = points_[r] - a;
			Point vr = points_[(r + 1) % n] - points_[r];
			float pointing_r = to_r ^ vr;
			if (FloatSign(pointing_r) > 0 && FloatSign(pointing_l) <= 0) {
				res.second = r;
				// Check if we should move the answer backward.
				// Should choose the previous vertex, if that previous point is "0" node,
				// but not the same vertex, which the first tangent is pointing at, unless |a| is equal to it.
				// But the equality case is already processed after the first binary search.
				if ((res.first != l) && FloatZero(pointing_l)) {
					res.second = l;
				}
			}
		}

		return res;
	}

	std::pair<int, int> Polygon::GetTangentIds(const Point& a) const
	{
		if ((int)points_.size() < kMinPointsForLogTangentsAlgo) {
			return  GetTangentIdsNaive(a);
		}
		else {
			return  GetTangentIdsLogarithmic(a);
		}
	}


	bool Polygon::IsTangent(int i, const Point& a) const
	{
		int n = (int)points_.size();
		Point v1 = points_[i] - points_[(i + n - 1) % n];
		Point v = a - points_[i];
		Point v2 = points_[(i + 1) % n] - points_[i];
		float dir1 = (v1 ^ v);
		float dir2 = (v ^ v2);

		// v lies between v1 and v2 or -v lies between v1 and v2.
		// Strict and unstrict checks allow only closest points
		// to be a tangent if |a| is on the continuation of the side.
		// Also, allow the point to be on one of the sides.
		// But not on both of them, since this would mean that the segment is
		// of a zero length. No sense adding this as an edge.
		return (FloatSign(dir1) >= 0 && FloatSign(dir2) > 0) ||
			(FloatSign(dir1) < 0 && FloatSign(dir2) <= 0) ||
			((FloatZero(dir2) && v.Len2() < v2.Len2()) ^
				(FloatZero(dir1) && v.Len2() < v1.Len2()));
	}

	Polygon::BoundaryResult Polygon::GetNearestBoundaryPoint(const Point& a) const
	{
		BoundaryResult result;
		result.nearest_point = Point(0, 0);
		result.edge_index = -1;
		result.distance = std::numeric_limits<float>::max();

		if (points_.empty()) {
			return result;
		}

		int n = static_cast<int>(points_.size());
		for (int i = 0; i < n; ++i) {
			const Point& p1 = points_[i];
			const Point& p2 = points_[(i + 1) % n];

			// Find nearest point on segment p1-p2 from point a
			Point edge = p2 - p1;
			float edge_len2 = edge.Len2();

			Point nearest;
			if (edge_len2 < EPSILON) {
				// Degenerate edge (p1 == p2)
				nearest = p1;
			} else {
				// Project a onto the line defined by the edge
				// t = ((a - p1) . edge) / |edge|^2
				float t = ((a - p1) * edge) / edge_len2;

				// Clamp t to [0, 1] to stay within the segment
				if (t <= 0.0f) {
					nearest = p1;
				} else if (t >= 1.0f) {
					nearest = p2;
				} else {
					nearest = p1 + edge * t;
				}
			}

			float dist = (a - nearest).Len();
			if (dist < result.distance) {
				result.distance = dist;
				result.nearest_point = nearest;
				result.edge_index = i;
			}
		}

		return result;
	}

}
