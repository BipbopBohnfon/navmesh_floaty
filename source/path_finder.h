#pragma once

#include <vector>
#include <map>
#include <utility>
#include <mutex>

#include "point.h"
#include "polygon.h"
#include "segment.h"
#include "spatial_grid.h"
#include "parallel.h"

namespace NavMesh {

	class PathFinder
	{
	public:
		// Call one time when map changes.
		// Resets the map and creates the graph.
		//
		// |polygons_to_add| - convex polygons on the map.
		// |inflate_by| - how far away paths must go from any polygon
		void AddPolygons(const std::vector<Polygon>& polygons_to_add, float inflate_by);

		// Call any time after AddPolygons().
		// It removes previously added external points and adds
		// |points| to the graph.
		void AddExternalPoints(const std::vector<Point>& points_);

		// Get shortest path between two points.
		// points must be first added via AddExternalPoints().
		std::vector<Point> GetPath(const Point& start_coord, const Point& dest_coord);

		// For debugging. Returns all the edges in the graph.
		std::vector<Segment> GetEdgesForDebug() const;

	private:
		int GetVertex(const Point& c);
		void AddEdge(int be, int en);

		// Original O(n) check - kept for compatibility
		bool CanAddSegment(const Segment& s, const std::vector<std::pair<int, int>>& tangents);

		// Optimized O(m) check using spatial grid where m << n
		bool CanAddSegmentOptimized(const Segment& s, const std::vector<std::pair<int, int>>& tangents);

		// Lazy tangent computation - computes tangents on-the-fly only for polygons along segment
		bool CanAddSegmentLazy(const Segment& s, const Point& from_point);
		bool CanAddSegmentLazyFallback(const Segment& s, const Point& from_point);

		// Compute AABB for a polygon
		AABB ComputePolygonAABB(const Polygon& p) const;

		// Build the spatial grid from current polygons
		void BuildSpatialGrid();

		std::vector<Polygon> polygons_;
		std::vector<AABB> polygon_aabbs_;  // Bounding boxes for each polygon
		std::vector<Point> ext_points_;

		std::vector<int> free_vertices_;
		std::map<Point, int> vertex_ids_;
		std::vector<Point> v_;

		std::vector<std::vector<bool>> polygon_point_is_inside_;

		std::vector<std::vector<std::pair<int, double>>> edges_;

		// Spatial partitioning for O(1) nearby polygon lookup
		SpatialGrid spatial_grid_;
		bool use_spatial_optimization_ = true;
		bool use_parallel_ = true;

		// Reusable buffer for spatial queries (avoid repeated allocations)
		mutable std::vector<int> query_buffer_;

		// Path caching - avoid recomputing path when source/dest unchanged
		mutable std::vector<Point> cached_path_;
		mutable Point cached_path_start_;
		mutable Point cached_path_dest_;
		mutable bool path_cache_valid_ = false;

		// Reusable A* vectors (avoid ~1.7MB allocation per GetPath call)
		mutable std::vector<int> astar_prev_;
		mutable std::vector<double> astar_dist_;
		mutable std::vector<double> astar_est_;
		mutable std::vector<bool> astar_done_;

		// Single mutex for all graph modifications (v_, edges_, vertex_ids_, free_vertices_)
		std::mutex graph_mutex_;

		// Thread-safe edge addition
		void AddEdgeThreadSafe(int be, int en);

		// Thread-safe vertex lookup/creation
		int GetVertexThreadSafe(const Point& c);

		// Lock-free vertex lookup (returns -1 if not found)
		// Safe when all vertices have been pre-allocated
		int GetVertexLockFree(const Point& c) const;
	};

}
