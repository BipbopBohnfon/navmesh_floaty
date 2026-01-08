#include "cone_of_vision.h"
#include "polygon.h"
#include "point.h"

#define _USE_MATH_DEFINES
#include <cmath>

namespace NavMesh
{
  std::vector<Point> ConeOfVision::GetVision(const Point& center, float radius,
    float max_angle, float start_angle, float angle_step)
  {
    std::vector<Point> vision;

    for (float angle = start_angle; angle < start_angle + max_angle; angle += angle_step)
    {
      Point vision_point(
        center.x + radius * std::cos(angle * (float)M_PI / 180.0f),
        center.y + radius * std::sin(angle * (float)M_PI / 180.0f));

      for (auto& polygon : polygons_)
      {
        for (int i = 0; i < polygon.Size(); i++)
        {
          Point p1(polygon[i]);
          Point p2(polygon[(i + 1) % polygon.Size()]);

          Segment vision_segment(center, vision_point);

          if (vision_segment.Intersects(p1, p2))
          {
            vision_point = vision_segment.GetIntersection(p1, p2);
          }
        }
      }

      vision.emplace_back(vision_point);
    }

    return vision;
  }

  void ConeOfVision::AddPolygons(const std::vector<Polygon>& polygons_to_add)
  {
    polygons_.clear();
    polygons_.reserve(polygons_to_add.size());
    for (auto const& p : polygons_to_add) {
      polygons_.emplace_back(p);
      // Don't add polygons which are not really an obstacle.
      if (polygons_.back().Size() < 1) polygons_.pop_back();
    }
  }
}
