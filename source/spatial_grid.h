#pragma once

#include "point.h"
#include "segment.h"

#include <vector>
#include <unordered_set>
#include <cmath>
#include <algorithm>

namespace NavMesh {

// Axis-Aligned Bounding Box for fast rejection tests
struct AABB {
    float min_x, min_y, max_x, max_y;

    AABB() : min_x(0), min_y(0), max_x(0), max_y(0) {}
    AABB(float minX, float minY, float maxX, float maxY)
        : min_x(minX), min_y(minY), max_x(maxX), max_y(maxY) {}

    bool Overlaps(const AABB& other) const {
        return !(max_x < other.min_x || min_x > other.max_x ||
                 max_y < other.min_y || min_y > other.max_y);
    }

    bool ContainsPoint(float x, float y) const {
        return x >= min_x && x <= max_x && y >= min_y && y <= max_y;
    }

    // Get AABB for a line segment (expanded slightly for safety)
    static AABB FromSegment(const Segment& s) {
        return AABB(
            std::min(s.b.x, s.e.x) - 0.001f,
            std::min(s.b.y, s.e.y) - 0.001f,
            std::max(s.b.x, s.e.x) + 0.001f,
            std::max(s.b.y, s.e.y) + 0.001f
        );
    }
};

// Spatial grid for O(1) lookup of nearby polygons
class SpatialGrid {
public:
    SpatialGrid() : cell_size_(100.0f), grid_width_(0), grid_height_(0) {}

    // Initialize grid with world bounds and cell size
    void Initialize(float min_x, float min_y, float max_x, float max_y, float cell_size) {
        min_x_ = min_x;
        min_y_ = min_y;
        max_x_ = max_x;
        max_y_ = max_y;
        cell_size_ = cell_size;

        grid_width_ = static_cast<int>(std::ceil((max_x - min_x) / cell_size)) + 1;
        grid_height_ = static_cast<int>(std::ceil((max_y - min_y) / cell_size)) + 1;

        cells_.clear();
        cells_.resize(grid_width_ * grid_height_);
    }

    // Clear all polygon references
    void Clear() {
        for (auto& cell : cells_) {
            cell.clear();
        }
    }

    // Add a polygon (by index) to all cells it overlaps
    void AddPolygon(int polygon_index, const AABB& bounds) {
        int min_cell_x = CellX(bounds.min_x);
        int max_cell_x = CellX(bounds.max_x);
        int min_cell_y = CellY(bounds.min_y);
        int max_cell_y = CellY(bounds.max_y);

        for (int cy = min_cell_y; cy <= max_cell_y; ++cy) {
            for (int cx = min_cell_x; cx <= max_cell_x; ++cx) {
                int cell_idx = CellIndex(cx, cy);
                if (cell_idx >= 0 && cell_idx < static_cast<int>(cells_.size())) {
                    cells_[cell_idx].push_back(polygon_index);
                }
            }
        }
    }

    // Get all polygon indices that could potentially intersect with a segment
    // Uses line rasterization to find cells along the segment path
    void GetPolygonsAlongSegment(const Segment& s, std::vector<int>& out_polygons) const {
        out_polygons.clear();

        // Use a set to avoid duplicates
        std::unordered_set<int> seen;

        // Get cells at start and end
        int x0 = CellX(s.b.x);
        int y0 = CellY(s.b.y);
        int x1 = CellX(s.e.x);
        int y1 = CellY(s.e.y);

        // Bresenham-style line rasterization through grid cells
        int dx = std::abs(x1 - x0);
        int dy = std::abs(y1 - y0);
        int sx = (x0 < x1) ? 1 : -1;
        int sy = (y0 < y1) ? 1 : -1;
        int err = dx - dy;

        int x = x0, y = y0;
        while (true) {
            // Add polygons from current cell and neighbors (for robustness)
            for (int ny = y - 1; ny <= y + 1; ++ny) {
                for (int nx = x - 1; nx <= x + 1; ++nx) {
                    int cell_idx = CellIndex(nx, ny);
                    if (cell_idx >= 0 && cell_idx < static_cast<int>(cells_.size())) {
                        for (int poly_idx : cells_[cell_idx]) {
                            if (seen.find(poly_idx) == seen.end()) {
                                seen.insert(poly_idx);
                                out_polygons.push_back(poly_idx);
                            }
                        }
                    }
                }
            }

            if (x == x1 && y == y1) break;

            int e2 = 2 * err;
            if (e2 > -dy) {
                err -= dy;
                x += sx;
            }
            if (e2 < dx) {
                err += dx;
                y += sy;
            }
        }
    }

    // Get all polygon indices in cells near a point
    void GetPolygonsNearPoint(const Point& p, std::vector<int>& out_polygons) const {
        out_polygons.clear();
        std::unordered_set<int> seen;

        int cx = CellX(p.x);
        int cy = CellY(p.y);

        // Check 3x3 neighborhood
        for (int ny = cy - 1; ny <= cy + 1; ++ny) {
            for (int nx = cx - 1; nx <= cx + 1; ++nx) {
                int cell_idx = CellIndex(nx, ny);
                if (cell_idx >= 0 && cell_idx < static_cast<int>(cells_.size())) {
                    for (int poly_idx : cells_[cell_idx]) {
                        if (seen.find(poly_idx) == seen.end()) {
                            seen.insert(poly_idx);
                            out_polygons.push_back(poly_idx);
                        }
                    }
                }
            }
        }
    }

    // Get polygons that could be visible from a point (within a radius)
    void GetPolygonsInRadius(const Point& p, float radius, std::vector<int>& out_polygons) const {
        out_polygons.clear();
        std::unordered_set<int> seen;

        int min_cx = CellX(p.x - radius);
        int max_cx = CellX(p.x + radius);
        int min_cy = CellY(p.y - radius);
        int max_cy = CellY(p.y + radius);

        for (int cy = min_cy; cy <= max_cy; ++cy) {
            for (int cx = min_cx; cx <= max_cx; ++cx) {
                int cell_idx = CellIndex(cx, cy);
                if (cell_idx >= 0 && cell_idx < static_cast<int>(cells_.size())) {
                    for (int poly_idx : cells_[cell_idx]) {
                        if (seen.find(poly_idx) == seen.end()) {
                            seen.insert(poly_idx);
                            out_polygons.push_back(poly_idx);
                        }
                    }
                }
            }
        }
    }

    // Get all unique polygon pairs that share at least one cell (for visibility computation)
    void GetNearbyPolygonPairs(std::vector<std::pair<int, int>>& out_pairs) const {
        out_pairs.clear();
        std::unordered_set<long long> seen_pairs;

        for (const auto& cell : cells_) {
            for (size_t i = 0; i < cell.size(); ++i) {
                for (size_t j = i + 1; j < cell.size(); ++j) {
                    int a = cell[i];
                    int b = cell[j];
                    if (a > b) std::swap(a, b);
                    long long key = (static_cast<long long>(a) << 32) | b;
                    if (seen_pairs.find(key) == seen_pairs.end()) {
                        seen_pairs.insert(key);
                        out_pairs.emplace_back(a, b);
                    }
                }
            }
        }
    }

    float GetCellSize() const { return cell_size_; }
    int GetGridWidth() const { return grid_width_; }
    int GetGridHeight() const { return grid_height_; }

private:
    int CellX(float x) const {
        return std::max(0, std::min(grid_width_ - 1,
            static_cast<int>((x - min_x_) / cell_size_)));
    }

    int CellY(float y) const {
        return std::max(0, std::min(grid_height_ - 1,
            static_cast<int>((y - min_y_) / cell_size_)));
    }

    int CellIndex(int cx, int cy) const {
        if (cx < 0 || cx >= grid_width_ || cy < 0 || cy >= grid_height_) {
            return -1;
        }
        return cy * grid_width_ + cx;
    }

    float min_x_, min_y_, max_x_, max_y_;
    float cell_size_;
    int grid_width_, grid_height_;
    std::vector<std::vector<int>> cells_;
};

} // namespace NavMesh
