#pragma once

#include <cmath>

namespace NavMesh {

	constexpr float EPSILON = 1e-6f;
	constexpr float SNAP_PRECISION = 1000.0f;  // Snap to 0.001 precision

	inline bool FloatEqual(float a, float b) {
		return std::abs(a - b) < EPSILON;
	}

	inline bool FloatZero(float a) {
		return std::abs(a) < EPSILON;
	}

	inline int FloatSign(float a) {
		if (a > EPSILON) return 1;
		if (a < -EPSILON) return -1;
		return 0;
	}

	class Point
	{
	public:
		Point() : x(0), y(0) {}
		Point(float x, float y) : x(x), y(y) {}
		Point(int x, int y) : x(static_cast<float>(x)), y(static_cast<float>(y)) {}

		Point& operator=(const Point& other) = default;

		Point operator+(const Point& other) const;
		Point operator-(const Point& other) const;

		// Scalar (dot) product.
		float operator*(const Point& other) const;

		// Cross product (z-component of 3D cross product).
		float operator^(const Point& other) const;

		// Scale by k.
		Point operator*(float k) const;

		bool operator==(const Point& other) const;
		bool operator!=(const Point& other) const;
		bool operator<(const Point& other) const;

		// Length of the vector.
		float Len() const;

		// Squared length.
		float Len2() const;

		// Returns a copy snapped to grid for stable map lookups.
		Point Snap() const;

		float x, y;
	};

}
