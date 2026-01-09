#include "path_finder.h"

#include <queue>
#include <algorithm>
#include <cmath>
#include <limits>

namespace NavMesh {

	AABB PathFinder::ComputePolygonAABB(const Polygon& p) const {
		if (p.Size() == 0) {
			return AABB();
		}

		float min_x = std::numeric_limits<float>::max();
		float min_y = std::numeric_limits<float>::max();
		float max_x = std::numeric_limits<float>::lowest();
		float max_y = std::numeric_limits<float>::lowest();

		for (int i = 0; i < p.Size(); ++i) {
			const Point& pt = p[i];
			min_x = std::min(min_x, pt.x);
			min_y = std::min(min_y, pt.y);
			max_x = std::max(max_x, pt.x);
			max_y = std::max(max_y, pt.y);
		}

		return AABB(min_x, min_y, max_x, max_y);
	}

	void PathFinder::BuildSpatialGrid() {
		if (polygons_.empty()) return;

		// Compute world bounds from all polygon AABBs
		float world_min_x = std::numeric_limits<float>::max();
		float world_min_y = std::numeric_limits<float>::max();
		float world_max_x = std::numeric_limits<float>::lowest();
		float world_max_y = std::numeric_limits<float>::lowest();

		for (const auto& aabb : polygon_aabbs_) {
			world_min_x = std::min(world_min_x, aabb.min_x);
			world_min_y = std::min(world_min_y, aabb.min_y);
			world_max_x = std::max(world_max_x, aabb.max_x);
			world_max_y = std::max(world_max_y, aabb.max_y);
		}

		// Calculate optimal cell size based on polygon density
		// Aim for roughly sqrt(n) cells on each axis for good distribution
		float world_width = world_max_x - world_min_x;
		float world_height = world_max_y - world_min_y;
		float world_size = std::max(world_width, world_height);

		// Cell size: aim for ~sqrt(n) cells per dimension
		// This gives O(sqrt(n)) polygons per cell on average for uniform distribution
		float target_cells = std::sqrt(static_cast<float>(polygons_.size()));
		float cell_size = world_size / std::max(1.0f, target_cells);

		// Clamp cell size to reasonable bounds
		cell_size = std::max(10.0f, std::min(cell_size, 1000.0f));

		// Initialize grid with some padding
		spatial_grid_.Initialize(
			world_min_x - cell_size,
			world_min_y - cell_size,
			world_max_x + cell_size,
			world_max_y + cell_size,
			cell_size
		);

		// Add all polygons to the grid
		for (size_t i = 0; i < polygons_.size(); ++i) {
			spatial_grid_.AddPolygon(static_cast<int>(i), polygon_aabbs_[i]);
		}
	}

	bool PathFinder::CanAddSegmentOptimized(const Segment& s, const std::vector<std::pair<int, int>>& tangents) {
		// Get segment AABB for fast rejection
		AABB seg_aabb = AABB::FromSegment(s);

		// Query spatial grid for polygons along the segment path
		spatial_grid_.GetPolygonsAlongSegment(s, query_buffer_);

		// Only check polygons that are near the segment
		for (int i : query_buffer_) {
			// Fast AABB rejection
			if (!seg_aabb.Overlaps(polygon_aabbs_[i])) {
				continue;
			}

			// Detailed intersection test
			if (polygons_[i].Intersects(s, tangents[i])) {
				return false;
			}
		}
		return true;
	}

	void PathFinder::AddEdgeThreadSafe(int be, int en) {
		std::lock_guard<std::mutex> lock(graph_mutex_);
		double dst = (v_[be] - v_[en]).Len();
		edges_[be].push_back(std::make_pair(en, dst));
		edges_[en].push_back(std::make_pair(be, dst));
	}

	void PathFinder::AddPolygons(const std::vector<Polygon>& polygons_to_add, float inflate_by = 0)
	{
		polygons_.clear();
		polygon_aabbs_.clear();
		v_.clear();
		edges_.clear();
		vertex_ids_.clear();
		polygons_.reserve(polygons_to_add.size());
		polygon_aabbs_.reserve(polygons_to_add.size());

		// Inflate polygons and compute their AABBs
		for (auto const& p : polygons_to_add) {
			Polygon inflated = p.Inflate(inflate_by);
			if (inflated.Size() < 1) continue;

			polygons_.emplace_back(std::move(inflated));
			polygon_aabbs_.push_back(ComputePolygonAABB(polygons_.back()));
		}

		if (polygons_.empty()) return;

		// Build spatial grid for fast queries
		BuildSpatialGrid();

		// Pre-initialize all polygons for IsInside queries to avoid race conditions
		// IsInside() lazily calls PrepareForFastInsideQueries() which modifies state
		for (auto& p : polygons_) {
			p.PrepareForFastInsideQueries();
		}

		auto& executor = GetParallelExecutor();

		// Calculate which polygon points are inside other polygons
		// PARALLEL: Each polygon can be processed independently
		polygon_point_is_inside_.resize(polygons_.size());
		for (size_t i = 0; i < polygons_.size(); ++i) {
			polygon_point_is_inside_[i].resize(polygons_[i].Size());
		}

		if (use_parallel_ && polygons_.size() > 100) {
			executor.ParallelFor(0, polygons_.size(), [this](size_t i) {
				std::vector<int> local_query_buffer;
				int n = polygons_[i].Size();

				for (int k = 0; k < n; ++k) {
					polygon_point_is_inside_[i][k] = false;
					const Point& cur_point = polygons_[i].points_[k];

					if (use_spatial_optimization_) {
						spatial_grid_.GetPolygonsNearPoint(cur_point, local_query_buffer);
						for (int j : local_query_buffer) {
							if (static_cast<size_t>(j) != i && polygons_[j].IsInside(cur_point)) {
								polygon_point_is_inside_[i][k] = true;
								break;
							}
						}
					} else {
						for (size_t j = 0; j < polygons_.size(); ++j) {
							if (i != j && polygons_[j].IsInside(cur_point)) {
								polygon_point_is_inside_[i][k] = true;
								break;
							}
						}
					}
				}
			});
		} else {
			// Sequential fallback for small polygon counts
			for (size_t i = 0; i < polygons_.size(); ++i) {
				int n = polygons_[i].Size();
				for (int k = 0; k < n; ++k) {
					polygon_point_is_inside_[i][k] = false;
					const Point& cur_point = polygons_[i].points_[k];

					if (use_spatial_optimization_) {
						spatial_grid_.GetPolygonsNearPoint(cur_point, query_buffer_);
						for (int j : query_buffer_) {
							if (static_cast<size_t>(j) != i && polygons_[j].IsInside(cur_point)) {
								polygon_point_is_inside_[i][k] = true;
								break;
							}
						}
					} else {
						for (size_t j = 0; j < polygons_.size(); ++j) {
							if (i != j && polygons_[j].IsInside(cur_point)) {
								polygon_point_is_inside_[i][k] = true;
								break;
							}
						}
					}
				}
			}
		}

		// Pre-allocate vertices for all polygon points to avoid lock contention
		// This is done sequentially as it modifies shared state
		for (size_t i = 0; i < polygons_.size(); ++i) {
			for (int k = 0; k < polygons_[i].Size(); ++k) {
				if (!polygon_point_is_inside_[i][k]) {
					GetVertex(polygons_[i].points_[k]);
				}
			}
		}

		// PARALLEL: Process each polygon's vertices in parallel
		// Each thread collects edges locally, then we merge at the end (avoids lock contention)
		if (use_parallel_ && polygons_.size() > 100) {
			// Use ParallelForWithLocal to collect edges per-thread, then merge
			executor.ParallelForWithLocal<std::vector<std::pair<int, int>>>(
				0, polygons_.size(),
				// Init: create empty local edge list
				[]() { return std::vector<std::pair<int, int>>(); },
				// Work: process polygon i, collect edges locally
				[this](size_t i, std::vector<std::pair<int, int>>& local_edges) {
					std::vector<std::pair<int, int>> tangents(polygons_.size());
					std::vector<int> segment_query_buffer;
					std::vector<int> radius_query_results;

					const auto& cur_poly = polygons_[i];
					int n = cur_poly.Size();

					for (int k = 0; k < n; ++k) {
						if (polygon_point_is_inside_[i][k]) {
							continue;
						}
						const Point& cur_point = cur_poly.points_[k];

						for (size_t j = 0; j < polygons_.size(); ++j) {
							tangents[j] = polygons_[j].GetTangentIds(cur_point);
						}

						// Add sides of polygon from this point
						Segment s = Segment(cur_point, cur_poly.points_[(k + 1) % n]);
						if (!polygon_point_is_inside_[i][(k + 1) % n]) {
							AABB seg_aabb = AABB::FromSegment(s);
							spatial_grid_.GetPolygonsAlongSegment(s, segment_query_buffer);

							bool can_add = true;
							for (int poly_idx : segment_query_buffer) {
								if (!seg_aabb.Overlaps(polygon_aabbs_[poly_idx])) continue;
								if (polygons_[poly_idx].Intersects(s, tangents[poly_idx])) {
									can_add = false;
									break;
								}
							}

							if (can_add) {
								// Lock-free vertex lookup (vertices pre-allocated)
								int be = GetVertexLockFree(s.b);
								int en = GetVertexLockFree(s.e);
								if (be >= 0 && en >= 0) {
									local_edges.emplace_back(be, en);
								}
							}
						}

						// Add tangents between polygons
						if (use_spatial_optimization_) {
							float max_dist = 0;
							for (size_t j = 0; j < polygons_.size() && j < 100; ++j) {
								float dx = polygon_aabbs_[j].max_x - cur_point.x;
								float dy = polygon_aabbs_[j].max_y - cur_point.y;
								max_dist = std::max(max_dist, std::sqrt(dx*dx + dy*dy));
							}
							float search_radius = std::min(max_dist, spatial_grid_.GetCellSize() * 20.0f);

							spatial_grid_.GetPolygonsInRadius(cur_point, search_radius, segment_query_buffer);
							radius_query_results = segment_query_buffer;

							for (int j_idx : radius_query_results) {
								size_t j = static_cast<size_t>(j_idx);
								if (j <= i) continue;

								const auto& other_poly = polygons_[j];
								const auto& ids = tangents[j];

								if (ids.first == ids.second) continue;

								if (ids.first >= 0 && cur_poly.IsTangent(k, other_poly.points_[ids.first])) {
									s = Segment(cur_poly.points_[k], other_poly.points_[ids.first]);
									if (!polygon_point_is_inside_[j][ids.first]) {
										AABB seg_aabb = AABB::FromSegment(s);
										spatial_grid_.GetPolygonsAlongSegment(s, segment_query_buffer);

										bool can_add = true;
										for (int poly_idx : segment_query_buffer) {
											if (!seg_aabb.Overlaps(polygon_aabbs_[poly_idx])) continue;
											if (polygons_[poly_idx].Intersects(s, tangents[poly_idx])) {
												can_add = false;
												break;
											}
										}

										if (can_add) {
											int be = GetVertexLockFree(s.b);
											int en = GetVertexLockFree(s.e);
											if (be >= 0 && en >= 0) {
												local_edges.emplace_back(be, en);
											}
										}
									}
								}
								if (ids.second >= 0 && cur_poly.IsTangent(k, other_poly.points_[ids.second])) {
									s = Segment(cur_poly.points_[k], other_poly.points_[ids.second]);
									if (!polygon_point_is_inside_[j][ids.second]) {
										AABB seg_aabb = AABB::FromSegment(s);
										spatial_grid_.GetPolygonsAlongSegment(s, segment_query_buffer);

										bool can_add = true;
										for (int poly_idx : segment_query_buffer) {
											if (!seg_aabb.Overlaps(polygon_aabbs_[poly_idx])) continue;
											if (polygons_[poly_idx].Intersects(s, tangents[poly_idx])) {
												can_add = false;
												break;
											}
										}

										if (can_add) {
											int be = GetVertexLockFree(s.b);
											int en = GetVertexLockFree(s.e);
											if (be >= 0 && en >= 0) {
												local_edges.emplace_back(be, en);
											}
										}
									}
								}
							}
						}
					}
				},
				// Merge: add all local edges to the graph (sequential, no locking needed)
				[this](std::vector<std::pair<int, int>>& local_edges) {
					for (const auto& edge : local_edges) {
						AddEdge(edge.first, edge.second);
					}
				}
			);
		} else {
			// Sequential fallback
			std::vector<std::pair<int, int>> tangents(polygons_.size());

			for (size_t i = 0; i < polygons_.size(); ++i) {
				const auto& cur_poly = polygons_[i];
				int n = cur_poly.Size();

				for (int k = 0; k < n; ++k) {
					if (polygon_point_is_inside_[i][k]) {
						continue;
					}
					const Point& cur_point = cur_poly.points_[k];

					for (size_t j = 0; j < polygons_.size(); ++j) {
						tangents[j] = polygons_[j].GetTangentIds(cur_point);
					}

					Segment s = Segment(cur_point, cur_poly.points_[(k + 1) % n]);
					if (!polygon_point_is_inside_[i][(k + 1) % n]) {
						bool can_add = use_spatial_optimization_
							? CanAddSegmentOptimized(s, tangents)
							: CanAddSegment(s, tangents);
						if (can_add) {
							AddEdge(GetVertex(s.b), GetVertex(s.e));
						}
					}

					if (use_spatial_optimization_) {
						float max_dist = 0;
						for (size_t j = 0; j < polygons_.size() && j < 100; ++j) {
							float dx = polygon_aabbs_[j].max_x - cur_point.x;
							float dy = polygon_aabbs_[j].max_y - cur_point.y;
							max_dist = std::max(max_dist, std::sqrt(dx*dx + dy*dy));
						}
						float search_radius = std::min(max_dist, spatial_grid_.GetCellSize() * 20.0f);

						spatial_grid_.GetPolygonsInRadius(cur_point, search_radius, query_buffer_);

						for (int j_idx : query_buffer_) {
							size_t j = static_cast<size_t>(j_idx);
							if (j <= i) continue;

							const auto& other_poly = polygons_[j];
							const auto& ids = tangents[j];

							if (ids.first == ids.second) continue;

							if (ids.first >= 0 && cur_poly.IsTangent(k, other_poly.points_[ids.first])) {
								s = Segment(cur_poly.points_[k], other_poly.points_[ids.first]);
								if (!polygon_point_is_inside_[j][ids.first] &&
									CanAddSegmentOptimized(s, tangents)) {
									AddEdge(GetVertex(s.b), GetVertex(s.e));
								}
							}
							if (ids.second >= 0 && cur_poly.IsTangent(k, other_poly.points_[ids.second])) {
								s = Segment(cur_poly.points_[k], other_poly.points_[ids.second]);
								if (!polygon_point_is_inside_[j][ids.second] &&
									CanAddSegmentOptimized(s, tangents)) {
									AddEdge(GetVertex(s.b), GetVertex(s.e));
								}
							}
						}
					} else {
						for (size_t j = i + 1; j < polygons_.size(); ++j) {
							const auto& other_poly = polygons_[j];
							const auto& ids = tangents[j];

							if (ids.first == ids.second) continue;

							if (ids.first >= 0 && cur_poly.IsTangent(k, other_poly.points_[ids.first])) {
								s = Segment(cur_poly.points_[k], other_poly.points_[ids.first]);
								if (!polygon_point_is_inside_[j][ids.first] && CanAddSegment(s, tangents)) {
									AddEdge(GetVertex(s.b), GetVertex(s.e));
								}
							}
							if (ids.second >= 0 && cur_poly.IsTangent(k, other_poly.points_[ids.second])) {
								s = Segment(cur_poly.points_[k], other_poly.points_[ids.second]);
								if (!polygon_point_is_inside_[j][ids.second] && CanAddSegment(s, tangents)) {
									AddEdge(GetVertex(s.b), GetVertex(s.e));
								}
							}
						}
					}
				}
			}
		}
	}

	void PathFinder::AddExternalPoints(const std::vector<Point>& points_)
	{
		// Remove old points
		for (const auto& p : ext_points_) {
			auto it = vertex_ids_.find(p.Snap());
			if (it == vertex_ids_.end()) continue;
			int id = it->second;
			free_vertices_.push_back(id);
			// Remove all edges from and to the point.
			for (const auto& e : edges_[id]) {
				int u = e.first;
				if (u == id) continue;
				size_t i;
				for (i = 0; i < edges_[u].size(); ++i) {
					if (edges_[u][i].first == id) {
						edges_[u][i] = edges_[u].back();
						edges_[u].resize(edges_[u].size() - 1);
						break;
					}
				}
			}
			edges_[id].clear();
			vertex_ids_.erase(it);
		}

		ext_points_ = points_;

		std::vector<std::pair<int, int>> tangents(polygons_.size());
		std::vector<bool> point_is_inside(points_.size(), false);

		// Check if external points are inside any polygon
		for (size_t i = 0; i < points_.size(); ++i) {
			if (use_spatial_optimization_ && !polygons_.empty()) {
				spatial_grid_.GetPolygonsNearPoint(points_[i], query_buffer_);
				for (int j : query_buffer_) {
					if (polygons_[j].IsInside(points_[i])) {
						point_is_inside[i] = true;
						break;
					}
				}
			} else {
				for (size_t j = 0; j < polygons_.size(); ++j) {
					if (polygons_[j].IsInside(points_[i])) {
						point_is_inside[i] = true;
						break;
					}
				}
			}
		}

		for (size_t i = 0; i < points_.size(); ++i) {
			const auto& cur_point = points_[i];
			if (point_is_inside[i]) continue;

			for (size_t j = 0; j < polygons_.size(); ++j) {
				tangents[j] = polygons_[j].GetTangentIds(cur_point);
			}

			// Add edges between external points
			for (size_t j = i + 1; j < points_.size(); ++j) {
				if (point_is_inside[j]) continue;

				Segment s(cur_point, points_[j]);
				bool can_add = use_spatial_optimization_
					? CanAddSegmentOptimized(s, tangents)
					: CanAddSegment(s, tangents);

				if (can_add) {
					AddEdge(GetVertex(cur_point), GetVertex(points_[j]));
				}
			}

			// Add tangents to polygons
			for (size_t j = 0; j < polygons_.size(); ++j) {
				const auto& ids = tangents[j];
				if (ids.first == -1 || ids.second == -1 || ids.first == ids.second)
					continue;

				const Point& other_point1 = polygons_[j].points_[ids.first];
				if (!polygon_point_is_inside_[j][ids.first]) {
					Segment s(cur_point, other_point1);
					bool can_add = use_spatial_optimization_
						? CanAddSegmentOptimized(s, tangents)
						: CanAddSegment(s, tangents);
					if (can_add) {
						AddEdge(GetVertex(cur_point), GetVertex(other_point1));
					}
				}

				const Point& other_point2 = polygons_[j].points_[ids.second];
				if (!polygon_point_is_inside_[j][ids.second]) {
					Segment s(cur_point, other_point2);
					bool can_add = use_spatial_optimization_
						? CanAddSegmentOptimized(s, tangents)
						: CanAddSegment(s, tangents);
					if (can_add) {
						AddEdge(GetVertex(cur_point), GetVertex(other_point2));
					}
				}
			}
		}
	}

	std::vector<Point> PathFinder::GetPath(const Point& start_coord, const Point& dest_coord)
	{
		int start = GetVertex(start_coord);
		int dest = GetVertex(dest_coord);

		if (start == dest) return { start_coord };

		// Run A*.
		std::vector<int> prev(v_.size(), -1);
		std::vector<double> dist(v_.size(), -1.0);
		std::vector<double> est(v_.size(), -1.0);
		std::vector<bool> done(v_.size(), false);
		std::priority_queue<std::pair<double, int>> queue;

		dist[start] = 0;
		est[start] = (v_[dest] - v_[start]).Len();
		queue.push(std::make_pair(-est[start], start));

		while (!queue.empty()) {
			int bst = queue.top().second;
			queue.pop();
			if (done[bst]) continue;
			done[bst] = true;
			if (bst == dest) break;

			for (const auto& e : edges_[bst]) {
				if (dist[e.first] < 0 || dist[e.first] > dist[bst] + e.second) {
					dist[e.first] = dist[bst] + e.second;
					est[e.first] = dist[e.first] + (v_[dest] - v_[e.first]).Len();
					queue.push(std::make_pair(-est[e.first], e.first));
					prev[e.first] = bst;
				}
			}
		}

		if (prev[dest] == -1) return {};

		std::vector<Point> res;
		int u = dest;
		while (u != start) {
			res.push_back(v_[u]);
			u = prev[u];
		}
		res.push_back(v_[start]);
		std::reverse(res.begin(), res.end());
		return res;
	}

	std::vector<Segment> PathFinder::GetEdgesForDebug() const
	{
		std::vector<Segment> res;
		for (int i = 0; i < (int)edges_.size(); ++i) {
			for (const auto& e : edges_[i]) {
				int j = e.first;
				if (j > i) {
					res.push_back(Segment(v_[i], v_[j]));
				}
			}
		}
		return res;
	}

	int PathFinder::GetVertex(const Point& c)
	{
		Point snapped = c.Snap();
		auto it = vertex_ids_.find(snapped);
		if (it != vertex_ids_.end()) {
			return it->second;
		}
		if (free_vertices_.empty()) {
			vertex_ids_[snapped] = (int)v_.size();
			v_.push_back(c);
			edges_.push_back({});
			return (int)v_.size() - 1;
		}
		else {
			int node = free_vertices_.back();
			free_vertices_.pop_back();
			v_[node] = c;
			vertex_ids_[snapped] = node;
			return node;
		}
	}

	int PathFinder::GetVertexThreadSafe(const Point& c)
	{
		std::lock_guard<std::mutex> lock(graph_mutex_);
		Point snapped = c.Snap();
		auto it = vertex_ids_.find(snapped);
		if (it != vertex_ids_.end()) {
			return it->second;
		}
		if (free_vertices_.empty()) {
			vertex_ids_[snapped] = (int)v_.size();
			v_.push_back(c);
			edges_.push_back({});
			return (int)v_.size() - 1;
		}
		else {
			int node = free_vertices_.back();
			free_vertices_.pop_back();
			v_[node] = c;
			vertex_ids_[snapped] = node;
			return node;
		}
	}

	int PathFinder::GetVertexLockFree(const Point& c) const
	{
		Point snapped = c.Snap();
		auto it = vertex_ids_.find(snapped);
		if (it != vertex_ids_.end()) {
			return it->second;
		}
		return -1;  // Not found - should not happen if vertices are pre-allocated
	}

	void PathFinder::AddEdge(int be, int en)
	{
		double dst = (v_[be] - v_[en]).Len();
		edges_[be].push_back(std::make_pair(en, dst));
		edges_[en].push_back(std::make_pair(be, dst));
	}

	bool PathFinder::CanAddSegment(const Segment& s, const std::vector<std::pair<int, int>>& tangents)
	{
		for (size_t i = 0; i < polygons_.size(); ++i) {
			if (polygons_[i].Intersects(s, tangents[i])) return false;
		}
		return true;
	}

}
