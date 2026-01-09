#include "point.h"
#include <cmath>

namespace NavMesh {

	Point Point::operator+(const Point& other) const
	{
		return Point(x + other.x, y + other.y);
	}

	Point Point::operator-(const Point& other) const
	{
		return Point(x - other.x, y - other.y);
	}

	// Scalar (dot) product.
	float Point::operator*(const Point& other) const
	{
		return x * other.x + y * other.y;
	}

	// Cross product (z-component of 3D cross product).
	float Point::operator^(const Point& other) const
	{
		return x * other.y - y * other.x;
	}

	Point Point::operator*(float k) const
	{
		return Point(x * k, y * k);
	}

	bool Point::operator==(const Point& other) const
	{
		return FloatEqual(x, other.x) && FloatEqual(y, other.y);
	}

	bool Point::operator!=(const Point& other) const
	{
		return !FloatEqual(x, other.x) || !FloatEqual(y, other.y);
	}

	bool Point::operator<(const Point& other) const
	{
		if (!FloatEqual(x, other.x)) return x < other.x;
		return y < other.y;
	}

	float Point::Len() const
	{
		return std::sqrt(x * x + y * y);
	}

	float Point::Len2() const
	{
		return x * x + y * y;
	}

}
