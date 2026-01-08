#pragma once

#include <vector>

#include "point.h"
#include "polygon.h"

namespace NavMesh
{
  class ConeOfVision
  {
  public:
    std::vector<Point> GetVision(const Point& center, float radius,
      float max_angle = 360.0f, float start_angle = 0.0f, float angle_step = 1.0f);
    void AddPolygons(const std::vector<Polygon>& polygons_to_add);

  private:
    std::vector<Polygon> polygons_;
  };
}
