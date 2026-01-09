#pragma once

#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include "point.h"
#include "polygon.h"
#include "segment.h"

namespace NavMesh {

	// Tangent pair: (left_tangent_vertex_id, right_tangent_vertex_id).
	using TangentPair = std::pair<int, int>;

	// Graph edge: (destination_vertex_id, edge_distance).
	using GraphEdge = std::pair<int, double>;

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
		void AddExternalPoints(const std::vector<Point>& points);

		// Get shortest path between two points.
		// Points must be first added via AddExternalPoints().
		std::vector<Point> GetPath(const Point& start_coord, const Point& dest_coord);

		// For debugging. Returns all the edges in the graph.
		std::vector<Segment> GetEdgesForDebug() const;

	private:
		// Returns the vertex ID for a point, creating a new vertex if needed.
		int GetOrCreateVertex(const Point& point);

		// Adds a bidirectional edge between two vertices.
		void AddBidirectionalEdge(int vertex_a, int vertex_b);

		// Checks if a segment can be added without intersecting any polygon.
		// Uses spatial grid to only check relevant polygons.
		bool IsSegmentValid(const Segment& segment) const;

		// Removes all edges connected to a vertex.
		void RemoveVertexEdges(int vertex_id);

		// Checks if a point is strictly inside any nearby polygon.
		bool IsPointInsideAnyPolygon(const Point& point) const;

		// Attempts to add a tangent edge from a polygon vertex to another polygon.
		void TryAddTangentEdge(
			const Point& from_point,
			int from_polygon_idx,
			int target_polygon_idx,
			int target_vertex_idx);

		// Returns the vertex ID for a point, or -1 if not found.
		int GetVertexId(const Point& point) const;

		// Returns all vertex IDs reachable from start_vertex via BFS.
		std::unordered_set<int> GetReachableVertices(int start_vertex) const;

		// Attempts to connect two disconnected components by finding valid tangent edges.
		// Returns true if a connecting edge was added.
		bool TryConnectComponents(
			const std::unordered_set<int>& component_a,
			const std::unordered_set<int>& component_b);

		// Removes unnecessary waypoints from a path by checking line-of-sight shortcuts.
		std::vector<Point> SmoothPath(const std::vector<Point>& path) const;

		// --- Spatial Grid ---

		// Builds the spatial grid from current polygons.
		void BuildSpatialGrid();

		// Converts world coordinates to grid cell indices.
		void WorldToGrid(float x, float y, int& cell_x, int& cell_y) const;

		// Gets the grid cell index from cell coordinates.
		size_t GetCellIndex(int cell_x, int cell_y) const;

		// Gets polygon indices in cells near a point (3x3 neighborhood).
		void GetNearbyPolygons(const Point& p, std::vector<size_t>& result) const;

		// Gets polygon indices along a segment's path through the grid.
		void GetPolygonsAlongSegment(const Point& start, const Point& end, std::vector<size_t>& result) const;

		// Gets polygon indices that could potentially interact with polygons near a point.
		// Uses a larger neighborhood for visibility graph construction.
		void GetPolygonsInRadius(const Point& p, int cell_radius, std::vector<size_t>& result) const;

		// Maximum cells to search in any direction for nearby polygons.
		static constexpr int kMaxSearchRadius = 6;

		// Target number of grid cells (will be adjusted based on world size).
		static constexpr int kTargetGridCells = 200;

		// Minimum cell size to avoid too many cells.
		static constexpr float kMinCellSize = 50.0f;

		// Spatial grid dimensions and origin.
		float grid_cell_size_ = 100.0f;
		float grid_origin_x_ = 0.0f;
		float grid_origin_y_ = 0.0f;
		int grid_width_ = 0;
		int grid_height_ = 0;

		// Grid cells containing polygon indices.
		std::vector<std::vector<size_t>> grid_cells_;

		// --- Polygon Data ---

		// Obstacle polygons (potentially inflated).
		std::vector<Polygon> polygons_;

		// External points (start/end locations).
		std::vector<Point> external_points_;

		// Pool of reusable vertex IDs.
		std::vector<int> free_vertex_ids_;

		// Map from point coordinates to vertex ID. O(1) average lookup.
		std::unordered_map<Point, int, PointHash> point_to_vertex_id_;

		// Vertex positions indexed by vertex ID.
		std::vector<Point> vertices_;

		// For each polygon vertex [i][j], true if it's inside another polygon.
		std::vector<std::vector<bool>> polygon_vertex_is_occluded_;

		// Adjacency list: edges_[v] contains all edges from vertex v.
		std::vector<std::vector<GraphEdge>> adjacency_list_;
	};


}