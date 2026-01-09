#pragma once

#include <cmath>
#include <functional>

namespace NavMesh {

	constexpr float EPSILON = 1e-6f;

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
		Point() : x(0.0f), y(0.0f) {}
		Point(float x, float y) : x(x), y(y) {}

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

		float x, y;
	};

	// Hash function for Point, enabling use in unordered_map/unordered_set.
	struct PointHash {
		size_t operator()(const Point& p) const noexcept {
			// Combine x and y hashes using bit mixing
			size_t hx = std::hash<float>{}(p.x);
			size_t hy = std::hash<float>{}(p.y);
			return hx ^ (hy << 1);
		}
	};

}
