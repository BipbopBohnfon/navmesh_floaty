#include "path_finder.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <queue>

namespace NavMesh {

	void PathFinder::AddPolygons(const std::vector<Polygon>& polygons_to_add, float inflate_by)
	{
		// Clear existing graph data.
		polygons_.clear();
		vertices_.clear();
		adjacency_list_.clear();
		point_to_vertex_id_.clear();
		grid_cells_.clear();

		// Add inflated polygons, filtering out degenerate ones.
		polygons_.reserve(polygons_to_add.size());
		for (const auto& polygon : polygons_to_add) {
			polygons_.emplace_back(polygon.Inflate(inflate_by));
			if (polygons_.back().Size() < 1) {
				polygons_.pop_back();
			}
		}

		if (polygons_.empty()) return;

		// Build spatial grid for efficient spatial queries.
		BuildSpatialGrid();

		// Pre-allocate graph structures.
		const size_t estimated_vertices = polygons_.size() * 15;
		vertices_.reserve(estimated_vertices);
		adjacency_list_.reserve(estimated_vertices);

		// Calculate which polygon vertices are occluded (inside other polygons).
		// Use spatial grid to only check nearby polygons.
		polygon_vertex_is_occluded_.resize(polygons_.size());
		std::vector<size_t> nearby_polygons;

		for (size_t poly_idx = 0; poly_idx < polygons_.size(); ++poly_idx) {
			const int vertex_count = polygons_[poly_idx].Size();
			polygon_vertex_is_occluded_[poly_idx].assign(vertex_count, false);

			for (int vertex_idx = 0; vertex_idx < vertex_count; ++vertex_idx) {
				const Point& vertex = polygons_[poly_idx].points_[vertex_idx];

				// Only check nearby polygons for occlusion.
				GetNearbyPolygons(vertex, nearby_polygons);

				for (size_t other_poly_idx : nearby_polygons) {
					if (poly_idx != other_poly_idx && polygons_[other_poly_idx].IsInside(vertex)) {
						polygon_vertex_is_occluded_[poly_idx][vertex_idx] = true;
						break;
					}
				}
			}
		}

		// Reusable buffer for nearby polygon queries.
		std::vector<size_t> polygons_in_radius;

		// Build visibility graph edges from polygon vertices.
		for (size_t poly_idx = 0; poly_idx < polygons_.size(); ++poly_idx) {
			const auto& polygon = polygons_[poly_idx];
			const int vertex_count = polygon.Size();

			for (int vertex_idx = 0; vertex_idx < vertex_count; ++vertex_idx) {
				if (polygon_vertex_is_occluded_[poly_idx][vertex_idx]) {
					continue;  // Skip vertices inside other polygons.
				}

				const Point& current_vertex = polygon.points_[vertex_idx];

				// Add edge to next vertex on polygon boundary (if valid).
				const int next_vertex_idx = (vertex_idx + 1) % vertex_count;
				if (!polygon_vertex_is_occluded_[poly_idx][next_vertex_idx]) {
					const Point& next_vertex = polygon.points_[next_vertex_idx];
					const Segment boundary_edge(current_vertex, next_vertex);
					if (IsSegmentValid(boundary_edge)) {
						AddBidirectionalEdge(GetOrCreateVertex(current_vertex), GetOrCreateVertex(next_vertex));
					}
				}

				// Get polygons within a radius for tangent edge consideration.
				// Use a larger radius to ensure we don't miss visibility connections.
				GetPolygonsInRadius(current_vertex, kMaxSearchRadius, polygons_in_radius);

				// Add tangent edges to other nearby polygons.
				for (size_t other_poly_idx : polygons_in_radius) {
					if (other_poly_idx <= poly_idx) continue;  // Avoid duplicates.

					const auto& other_polygon = polygons_[other_poly_idx];
					auto tangent_ids = other_polygon.GetTangentIds(current_vertex);

					// Skip if current vertex coincides with other polygon vertex.
					if (tangent_ids.first == tangent_ids.second) continue;

					// Try left tangent.
					if (tangent_ids.first >= 0 && polygon.IsTangent(vertex_idx, other_polygon.points_[tangent_ids.first])) {
						TryAddTangentEdge(current_vertex, poly_idx, other_poly_idx, tangent_ids.first);
					}

					// Try right tangent.
					if (tangent_ids.second >= 0 && polygon.IsTangent(vertex_idx, other_polygon.points_[tangent_ids.second])) {
						TryAddTangentEdge(current_vertex, poly_idx, other_poly_idx, tangent_ids.second);
					}
				}
			}
		}
	}

	void PathFinder::AddExternalPoints(const std::vector<Point>& points)
	{
		// Remove edges from previous external points.
		for (const auto& old_point : external_points_) {
			auto it = point_to_vertex_id_.find(old_point);
			if (it == point_to_vertex_id_.end()) continue;

			const int vertex_id = it->second;
			free_vertex_ids_.push_back(vertex_id);
			RemoveVertexEdges(vertex_id);
			point_to_vertex_id_.erase(it);
		}

		external_points_ = points;

		// Pre-compute which external points are inside polygons.
		std::vector<bool> point_is_inside(points.size(), false);
		for (size_t i = 0; i < points.size(); ++i) {
			point_is_inside[i] = IsPointInsideAnyPolygon(points[i]);
		}

		// Reusable buffer for polygon queries.
		std::vector<size_t> polygons_in_radius;

		// Add edges for valid external points.
		for (size_t i = 0; i < points.size(); ++i) {
			if (point_is_inside[i]) continue;

			const Point& current_point = points[i];

			// Connect to other external points.
			for (size_t j = i + 1; j < points.size(); ++j) {
				if (!point_is_inside[j]) {
					const Segment edge(current_point, points[j]);
					if (IsSegmentValid(edge)) {
						AddBidirectionalEdge(GetOrCreateVertex(current_point), GetOrCreateVertex(points[j]));
					} else {
						// Direct path is blocked. Find polygons along the path and connect
						// to their tangent points so we can route around them.
						std::vector<size_t> blocking_polygons;
						GetPolygonsAlongSegment(current_point, points[j], blocking_polygons);

						for (size_t poly_idx : blocking_polygons) {
							const auto& polygon = polygons_[poly_idx];

							// Connect current_point to this polygon's tangents.
							auto tangent_ids_i = polygon.GetTangentIds(current_point);
							if (tangent_ids_i.first >= 0 && tangent_ids_i.first != tangent_ids_i.second) {
								if (!polygon_vertex_is_occluded_[poly_idx][tangent_ids_i.first]) {
									const Point& tangent_point = polygon.points_[tangent_ids_i.first];
									const Segment tangent_edge(current_point, tangent_point);
									if (IsSegmentValid(tangent_edge)) {
										AddBidirectionalEdge(GetOrCreateVertex(current_point), GetOrCreateVertex(tangent_point));
									}
								}
								if (!polygon_vertex_is_occluded_[poly_idx][tangent_ids_i.second]) {
									const Point& tangent_point = polygon.points_[tangent_ids_i.second];
									const Segment tangent_edge(current_point, tangent_point);
									if (IsSegmentValid(tangent_edge)) {
										AddBidirectionalEdge(GetOrCreateVertex(current_point), GetOrCreateVertex(tangent_point));
									}
								}
							}

							// Connect points[j] to this polygon's tangents.
							auto tangent_ids_j = polygon.GetTangentIds(points[j]);
							if (tangent_ids_j.first >= 0 && tangent_ids_j.first != tangent_ids_j.second) {
								if (!polygon_vertex_is_occluded_[poly_idx][tangent_ids_j.first]) {
									const Point& tangent_point = polygon.points_[tangent_ids_j.first];
									const Segment tangent_edge(points[j], tangent_point);
									if (IsSegmentValid(tangent_edge)) {
										AddBidirectionalEdge(GetOrCreateVertex(points[j]), GetOrCreateVertex(tangent_point));
									}
								}
								if (!polygon_vertex_is_occluded_[poly_idx][tangent_ids_j.second]) {
									const Point& tangent_point = polygon.points_[tangent_ids_j.second];
									const Segment tangent_edge(points[j], tangent_point);
									if (IsSegmentValid(tangent_edge)) {
										AddBidirectionalEdge(GetOrCreateVertex(points[j]), GetOrCreateVertex(tangent_point));
									}
								}
							}
						}
					}
				}
			}

			// Get nearby polygons for tangent connections.
			GetPolygonsInRadius(current_point, kMaxSearchRadius, polygons_in_radius);

			// Connect to polygon tangent points.
			for (size_t poly_idx : polygons_in_radius) {
				const auto& polygon = polygons_[poly_idx];
				auto tangent_ids = polygon.GetTangentIds(current_point);

				if (tangent_ids.first == -1 || tangent_ids.second == -1 || tangent_ids.first == tangent_ids.second) {
					continue;
				}

				// Try left tangent.
				if (!polygon_vertex_is_occluded_[poly_idx][tangent_ids.first]) {
					const Point& left_tangent_point = polygon.points_[tangent_ids.first];
					const Segment edge(current_point, left_tangent_point);
					if (IsSegmentValid(edge)) {
						AddBidirectionalEdge(GetOrCreateVertex(current_point), GetOrCreateVertex(left_tangent_point));
					}
				}

				// Try right tangent.
				if (!polygon_vertex_is_occluded_[poly_idx][tangent_ids.second]) {
					const Point& right_tangent_point = polygon.points_[tangent_ids.second];
					const Segment edge(current_point, right_tangent_point);
					if (IsSegmentValid(edge)) {
						AddBidirectionalEdge(GetOrCreateVertex(current_point), GetOrCreateVertex(right_tangent_point));
					}
				}
			}
		}
	}

	std::vector<Point> PathFinder::GetPath(const Point& start_coord, const Point& dest_coord)
	{
		const int start = GetOrCreateVertex(start_coord);
		const int dest = GetOrCreateVertex(dest_coord);

		if (start == dest) {
			return { start_coord };
		}

		// A* search.
		const size_t vertex_count = vertices_.size();
		std::vector<int> predecessor(vertex_count, -1);
		std::vector<double> distance(vertex_count, -1.0);
		std::vector<double> estimated_total(vertex_count, -1.0);
		std::vector<bool> finalized(vertex_count, false);

		// Max-heap with negated priorities for min-heap behavior.
		std::priority_queue<std::pair<double, int>> open_set;

		distance[start] = 0;
		estimated_total[start] = (vertices_[dest] - vertices_[start]).Len();
		open_set.push(std::make_pair(-estimated_total[start], start));

		while (!open_set.empty()) {
			const int current = open_set.top().second;
			open_set.pop();

			if (finalized[current]) continue;
			finalized[current] = true;

			if (current == dest) break;

			for (const auto& edge : adjacency_list_[current]) {
				const int neighbor = edge.first;
				const double edge_distance = edge.second;
				const double tentative_distance = distance[current] + edge_distance;

				if (distance[neighbor] < 0 || tentative_distance < distance[neighbor]) {
					distance[neighbor] = tentative_distance;
					estimated_total[neighbor] = tentative_distance + (vertices_[dest] - vertices_[neighbor]).Len();
					open_set.push(std::make_pair(-estimated_total[neighbor], neighbor));
					predecessor[neighbor] = current;
				}
			}
		}

		if (predecessor[dest] == -1) {
			// Path not found. Check if start and dest are in disconnected components.
			auto reachable_from_start = GetReachableVertices(start);
			if (reachable_from_start.find(dest) == reachable_from_start.end()) {
				// They are disconnected. Try to connect the components.
				auto reachable_from_dest = GetReachableVertices(dest);
				if (TryConnectComponents(reachable_from_start, reachable_from_dest)) {
					// Successfully added a connecting edge. Retry pathfinding.
					return GetPath(start_coord, dest_coord);
				}
			}
			return {};  // No path exists.
		}

		// Reconstruct path from destination to start.
		std::vector<Point> path;
		for (int current = dest; current != start; current = predecessor[current]) {
			path.push_back(vertices_[current]);
		}
		path.push_back(vertices_[start]);
		std::reverse(path.begin(), path.end());

		return SmoothPath(path);
	}

	std::vector<Segment> PathFinder::GetEdgesForDebug() const
	{
		std::vector<Segment> result;
		for (int i = 0; i < static_cast<int>(adjacency_list_.size()); ++i) {
			for (const auto& edge : adjacency_list_[i]) {
				const int j = edge.first;
				if (j > i) {  // Avoid duplicates (edges are bidirectional).
					result.emplace_back(vertices_[i], vertices_[j]);
				}
			}
		}
		return result;
	}

	// --- Spatial Grid Implementation ---

	void PathFinder::BuildSpatialGrid()
	{
		if (polygons_.empty()) {
			grid_width_ = 0;
			grid_height_ = 0;
			grid_cells_.clear();
			return;
		}

		// Find world bounds from all polygon bounding boxes.
		float min_x = std::numeric_limits<float>::max();
		float max_x = std::numeric_limits<float>::lowest();
		float min_y = std::numeric_limits<float>::max();
		float max_y = std::numeric_limits<float>::lowest();

		for (const auto& polygon : polygons_) {
			const AABB& bbox = polygon.GetBoundingBox();
			min_x = std::min(min_x, bbox.min_x);
			max_x = std::max(max_x, bbox.max_x);
			min_y = std::min(min_y, bbox.min_y);
			max_y = std::max(max_y, bbox.max_y);
		}

		// Calculate adaptive cell size based on world size.
		float world_width = max_x - min_x;
		float world_height = max_y - min_y;
		float world_diagonal = std::sqrt(world_width * world_width + world_height * world_height);

		// Target approximately kTargetGridCells cells across the diagonal.
		grid_cell_size_ = std::max(kMinCellSize, world_diagonal / kTargetGridCells);

		// Add padding to ensure points on the boundary are handled.
		min_x -= grid_cell_size_;
		min_y -= grid_cell_size_;
		max_x += grid_cell_size_;
		max_y += grid_cell_size_;

		grid_origin_x_ = min_x;
		grid_origin_y_ = min_y;

		// Calculate grid dimensions.
		grid_width_ = static_cast<int>(std::ceil((max_x - min_x) / grid_cell_size_));
		grid_height_ = static_cast<int>(std::ceil((max_y - min_y) / grid_cell_size_));

		// Clamp to reasonable size.
		grid_width_ = std::max(1, std::min(grid_width_, 500));
		grid_height_ = std::max(1, std::min(grid_height_, 500));

		// Initialize grid cells.
		grid_cells_.clear();
		grid_cells_.resize(static_cast<size_t>(grid_width_) * grid_height_);

		// Assign each polygon to all cells its bounding box overlaps.
		for (size_t poly_idx = 0; poly_idx < polygons_.size(); ++poly_idx) {
			const AABB& bbox = polygons_[poly_idx].GetBoundingBox();

			int min_cell_x, min_cell_y, max_cell_x, max_cell_y;
			WorldToGrid(bbox.min_x, bbox.min_y, min_cell_x, min_cell_y);
			WorldToGrid(bbox.max_x, bbox.max_y, max_cell_x, max_cell_y);

			for (int cy = min_cell_y; cy <= max_cell_y; ++cy) {
				for (int cx = min_cell_x; cx <= max_cell_x; ++cx) {
					size_t cell_idx = GetCellIndex(cx, cy);
					grid_cells_[cell_idx].push_back(poly_idx);
				}
			}
		}
	}

	void PathFinder::WorldToGrid(float x, float y, int& cell_x, int& cell_y) const
	{
		cell_x = static_cast<int>((x - grid_origin_x_) / grid_cell_size_);
		cell_y = static_cast<int>((y - grid_origin_y_) / grid_cell_size_);

		// Clamp to grid bounds.
		cell_x = std::max(0, std::min(cell_x, grid_width_ - 1));
		cell_y = std::max(0, std::min(cell_y, grid_height_ - 1));
	}

	size_t PathFinder::GetCellIndex(int cell_x, int cell_y) const
	{
		return static_cast<size_t>(cell_y) * grid_width_ + cell_x;
	}

	void PathFinder::GetNearbyPolygons(const Point& p, std::vector<size_t>& result) const
	{
		result.clear();

		if (grid_cells_.empty()) return;

		int center_x, center_y;
		WorldToGrid(p.x, p.y, center_x, center_y);

		// Check 3x3 neighborhood.
		for (int dy = -1; dy <= 1; ++dy) {
			for (int dx = -1; dx <= 1; ++dx) {
				int cx = center_x + dx;
				int cy = center_y + dy;

				if (cx < 0 || cx >= grid_width_ || cy < 0 || cy >= grid_height_) {
					continue;
				}

				const auto& cell = grid_cells_[GetCellIndex(cx, cy)];
				for (size_t poly_idx : cell) {
					result.push_back(poly_idx);
				}
			}
		}

		// Remove duplicates efficiently: O(n log n) sort + O(n) unique.
		std::sort(result.begin(), result.end());
		result.erase(std::unique(result.begin(), result.end()), result.end());
	}

	void PathFinder::GetPolygonsInRadius(const Point& p, int cell_radius, std::vector<size_t>& result) const
	{
		result.clear();

		if (grid_cells_.empty()) return;

		int center_x, center_y;
		WorldToGrid(p.x, p.y, center_x, center_y);

		// Check cells within radius.
		for (int dy = -cell_radius; dy <= cell_radius; ++dy) {
			for (int dx = -cell_radius; dx <= cell_radius; ++dx) {
				int cx = center_x + dx;
				int cy = center_y + dy;

				if (cx < 0 || cx >= grid_width_ || cy < 0 || cy >= grid_height_) {
					continue;
				}

				const auto& cell = grid_cells_[GetCellIndex(cx, cy)];
				for (size_t poly_idx : cell) {
					result.push_back(poly_idx);
				}
			}
		}

		// Remove duplicates efficiently: O(n log n) sort + O(n) unique.
		std::sort(result.begin(), result.end());
		result.erase(std::unique(result.begin(), result.end()), result.end());
	}

	void PathFinder::GetPolygonsAlongSegment(const Point& start, const Point& end, std::vector<size_t>& result) const
	{
		result.clear();

		if (grid_cells_.empty()) return;

		int start_x, start_y, end_x, end_y;
		WorldToGrid(start.x, start.y, start_x, start_y);
		WorldToGrid(end.x, end.y, end_x, end_y);

		// Use DDA algorithm to traverse cells along the segment.
		int dx = end_x - start_x;
		int dy = end_y - start_y;
		int steps = std::max(std::abs(dx), std::abs(dy));

		if (steps == 0) {
			// Start and end in same cell.
			const auto& cell = grid_cells_[GetCellIndex(start_x, start_y)];
			result.insert(result.end(), cell.begin(), cell.end());
			return;
		}

		float x_inc = static_cast<float>(dx) / steps;
		float y_inc = static_cast<float>(dy) / steps;

		float x = static_cast<float>(start_x);
		float y = static_cast<float>(start_y);

		// Track visited cells to avoid processing the same cell multiple times.
		std::vector<bool> visited(grid_cells_.size(), false);

		for (int i = 0; i <= steps; ++i) {
			int cx = static_cast<int>(std::round(x));
			int cy = static_cast<int>(std::round(y));

			cx = std::max(0, std::min(cx, grid_width_ - 1));
			cy = std::max(0, std::min(cy, grid_height_ - 1));

			size_t cell_idx = GetCellIndex(cx, cy);

			if (!visited[cell_idx]) {
				visited[cell_idx] = true;

				// Also check adjacent cells to catch edge cases.
				for (int adj_dy = -1; adj_dy <= 1; ++adj_dy) {
					for (int adj_dx = -1; adj_dx <= 1; ++adj_dx) {
						int adj_cx = cx + adj_dx;
						int adj_cy = cy + adj_dy;

						if (adj_cx < 0 || adj_cx >= grid_width_ || adj_cy < 0 || adj_cy >= grid_height_) {
							continue;
						}

						size_t adj_cell_idx = GetCellIndex(adj_cx, adj_cy);
						if (!visited[adj_cell_idx]) {
							visited[adj_cell_idx] = true;
						}

						const auto& cell = grid_cells_[adj_cell_idx];
						for (size_t poly_idx : cell) {
							result.push_back(poly_idx);
						}
					}
				}
			}

			x += x_inc;
			y += y_inc;
		}

		// Remove duplicates efficiently: O(n log n) sort + O(n) unique.
		std::sort(result.begin(), result.end());
		result.erase(std::unique(result.begin(), result.end()), result.end());
	}

	// --- Graph Building Helpers ---

	int PathFinder::GetOrCreateVertex(const Point& point)
	{
		auto it = point_to_vertex_id_.find(point);
		if (it != point_to_vertex_id_.end()) {
			return it->second;
		}

		if (free_vertex_ids_.empty()) {
			const int new_id = static_cast<int>(vertices_.size());
			point_to_vertex_id_[point] = new_id;
			vertices_.push_back(point);
			adjacency_list_.emplace_back();
			return new_id;
		}

		const int reused_id = free_vertex_ids_.back();
		free_vertex_ids_.pop_back();
		vertices_[reused_id] = point;
		point_to_vertex_id_[point] = reused_id;
		return reused_id;
	}

	void PathFinder::AddBidirectionalEdge(int vertex_a, int vertex_b)
	{
		const double distance = (vertices_[vertex_a] - vertices_[vertex_b]).Len();
		adjacency_list_[vertex_a].emplace_back(vertex_b, distance);
		adjacency_list_[vertex_b].emplace_back(vertex_a, distance);
	}

	bool PathFinder::IsSegmentValid(const Segment& segment) const
	{
		// Get all polygons along the segment's path.
		std::vector<size_t> polygons_to_check;
		GetPolygonsAlongSegment(segment.b, segment.e, polygons_to_check);

		// Check intersection with each polygon.
		// We compute tangents from segment start to properly handle edge cases
		// where the segment passes through polygon vertices.
		for (size_t poly_idx : polygons_to_check) {
			const auto& polygon = polygons_[poly_idx];
			auto tangents = polygon.GetTangentIds(segment.b);
			if (polygon.Intersects(segment, tangents)) {
				return false;
			}
		}
		return true;
	}

	void PathFinder::RemoveVertexEdges(int vertex_id)
	{
		// Remove edges pointing to this vertex from neighbors.
		for (const auto& edge : adjacency_list_[vertex_id]) {
			const int neighbor_id = edge.first;
			if (neighbor_id == vertex_id) continue;

			auto& neighbor_edges = adjacency_list_[neighbor_id];
			for (size_t i = 0; i < neighbor_edges.size(); ++i) {
				if (neighbor_edges[i].first == vertex_id) {
					neighbor_edges[i] = neighbor_edges.back();
					neighbor_edges.pop_back();
					break;
				}
			}
		}
		adjacency_list_[vertex_id].clear();
	}

	bool PathFinder::IsPointInsideAnyPolygon(const Point& point) const
	{
		// Only check nearby polygons.
		std::vector<size_t> nearby;
		GetNearbyPolygons(point, nearby);

		for (size_t poly_idx : nearby) {
			if (polygons_[poly_idx].IsInside(point)) {
				return true;
			}
		}
		return false;
	}

	void PathFinder::TryAddTangentEdge(
		const Point& from_point,
		int from_polygon_idx,
		int target_polygon_idx,
		int target_vertex_idx)
	{
		if (polygon_vertex_is_occluded_[target_polygon_idx][target_vertex_idx]) {
			return;
		}

		const Point& target_point = polygons_[target_polygon_idx].points_[target_vertex_idx];
		const Segment edge(from_point, target_point);

		if (IsSegmentValid(edge)) {
			AddBidirectionalEdge(GetOrCreateVertex(from_point), GetOrCreateVertex(target_point));
		}
	}

	int PathFinder::GetVertexId(const Point& point) const
	{
		auto it = point_to_vertex_id_.find(point);
		if (it != point_to_vertex_id_.end()) {
			return it->second;
		}
		return -1;
	}

	std::unordered_set<int> PathFinder::GetReachableVertices(int start_vertex) const
	{
		std::unordered_set<int> reachable;
		if (start_vertex < 0 || start_vertex >= static_cast<int>(adjacency_list_.size())) {
			return reachable;
		}

		std::queue<int> frontier;
		frontier.push(start_vertex);
		reachable.insert(start_vertex);

		while (!frontier.empty()) {
			const int current = frontier.front();
			frontier.pop();

			for (const auto& edge : adjacency_list_[current]) {
				const int neighbor = edge.first;
				if (reachable.find(neighbor) == reachable.end()) {
					reachable.insert(neighbor);
					frontier.push(neighbor);
				}
			}
		}

		return reachable;
	}

	bool PathFinder::TryConnectComponents(
		const std::unordered_set<int>& component_a,
		const std::unordered_set<int>& component_b)
	{
		// Find shortest valid edge between components by checking tangents to all polygons.
		double best_distance = std::numeric_limits<double>::max();
		int best_a = -1;
		int best_b = -1;

		for (int vertex_a : component_a) {
			const Point& point_a = vertices_[vertex_a];

			// Check tangents to ALL polygons (no radius limit).
			for (size_t poly_idx = 0; poly_idx < polygons_.size(); ++poly_idx) {
				const auto& polygon = polygons_[poly_idx];
				auto tangent_ids = polygon.GetTangentIds(point_a);

				if (tangent_ids.first < 0 || tangent_ids.first == tangent_ids.second) {
					continue;
				}

				for (int tid : {tangent_ids.first, tangent_ids.second}) {
					if (tid < 0 || polygon_vertex_is_occluded_[poly_idx][tid]) {
						continue;
					}

					const Point& tangent_point = polygon.points_[tid];
					int vertex_b = GetVertexId(tangent_point);

					if (vertex_b < 0 || component_b.find(vertex_b) == component_b.end()) {
						continue;
					}

					Segment edge(point_a, tangent_point);
					if (!IsSegmentValid(edge)) {
						continue;
					}

					double dist = (tangent_point - point_a).Len();
					if (dist < best_distance) {
						best_distance = dist;
						best_a = vertex_a;
						best_b = vertex_b;
					}
				}
			}
		}

		if (best_a >= 0 && best_b >= 0) {
			AddBidirectionalEdge(best_a, best_b);
			return true;
		}
		return false;
	}

	std::vector<Point> PathFinder::SmoothPath(const std::vector<Point>& path) const
	{
		if (path.size() <= 2) {
			return path;
		}

		std::vector<Point> smoothed;
		smoothed.push_back(path[0]);

		size_t current = 0;
		while (current < path.size() - 1) {
			// Find furthest reachable point from current position.
			size_t furthest = current + 1;
			for (size_t i = path.size() - 1; i > current + 1; --i) {
				Segment shortcut(path[current], path[i]);
				if (IsSegmentValid(shortcut)) {
					furthest = i;
					break;
				}
			}
			smoothed.push_back(path[furthest]);
			current = furthest;
		}

		return smoothed;
	}

}
