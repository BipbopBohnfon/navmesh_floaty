# NavMesh

This is a small and fast library for pathfinding in 2D space around convex polygonal obstacles. This may be useful in GameDev.
The implementation can pass around the obstacles at a given distance (if the actor is not a zero-width point, but has some physical size).
The project also contains a little demo.

![Example of the demo output](https://github.com/ilyanikolaevsky/navmesh/blob/master/picture.png?raw=true)


The main feature of this library is its speed: for a moderate amount (100) of quite big polygons (20 points each) the library computes
everything in 7-8ms on a i7-6700k CPU. It doesn't require baking in the graph around the geometry, which allows compeletely dynamic geometry of the map.

## Usage

You need to include ``path_finder.h``, ``point.h``, ``segment.h``, ``polygon.h`` files in your project and link against the static library.
Alternatively, since it's a small project, you can just add all files from ``source/`` to your project.

### Core Classes

The main class is ``NavMesh::PathFinder``. Supporting classes include:
- ``NavMesh::Point`` - 2D point/vector with arithmetic operations
- ``NavMesh::Segment`` - Line segment with intersection testing
- ``NavMesh::Polygon`` - Convex polygon with tangent and containment queries

### Type Aliases

The library provides type aliases for common data structures:
- ``NavMesh::TangentPair`` - Pair of tangent vertex indices ``(left_id, right_id)``
- ``NavMesh::GraphEdge`` - Graph edge as ``(destination_vertex_id, distance)``

### API Reference

#### PathFinder::AddPolygons
```cpp
void AddPolygons(const std::vector<Polygon>& polygons_to_add, float inflate_by);
```
Call each time the map changes. Builds the visibility graph from obstacle polygons.
- ``polygons_to_add`` - Convex polygons representing obstacles
- ``inflate_by`` - Minimum clearance distance from obstacles (set to actor radius)

**Complexity:** ``O(n^3*k)`` time, ``O(n^2*k)`` memory worst case, where ``n`` is the number of polygons and ``k`` is average vertices per polygon.

#### PathFinder::AddExternalPoints
```cpp
void AddExternalPoints(const std::vector<Point>& points);
```
Call after ``AddPolygons`` when start/destination points change. Connects external points to the visibility graph.
- ``points`` - Query points (start positions, destinations)

**Complexity:** ``O(p*n^2)`` time, ``O(n*p)`` memory, where ``p`` is the number of points.

#### PathFinder::GetPath
```cpp
std::vector<Point> GetPath(const Point& start_coord, const Point& dest_coord);
```
Finds the shortest path between two points using A* search. Points must have been added via ``AddExternalPoints``.
- Returns: Ordered vector of waypoints from start to destination, or empty vector if no path exists.

**Complexity:** ``O((n*k+p)*log(n*k+p))`` time, ``O(n*k+p)`` memory.

#### PathFinder::GetEdgesForDebug
```cpp
std::vector<Segment> GetEdgesForDebug() const;
```
Returns all edges in the visibility graph for debugging/visualization.

### Example

```cpp
#include "path_finder.h"
#include "polygon.h"
#include "point.h"

using namespace NavMesh;

int main() {
    // Create an obstacle polygon
    Polygon obstacle;
    obstacle.AddPoint(100.0f, 100.0f);
    obstacle.AddPoint(200.0f, 100.0f);
    obstacle.AddPoint(200.0f, 200.0f);
    obstacle.AddPoint(100.0f, 200.0f);

    // Initialize pathfinder with obstacles
    PathFinder pathfinder;
    pathfinder.AddPolygons({obstacle}, 10.0f);  // 10 unit clearance

    // Add start and destination points
    Point start(50.0f, 150.0f);
    Point destination(250.0f, 150.0f);
    pathfinder.AddExternalPoints({start, destination});

    // Find path
    std::vector<Point> path = pathfinder.GetPath(start, destination);

    // path now contains waypoints from start to destination
    // avoiding the obstacle with 10 unit clearance
    return 0;
}
```

## Details

This project constructs the visibility graph around obstacles using polygon sides and tangents to polygons as edges in the graph.
Then it uses A* on the constructed graph to find the shortest path. This implementation takes 8ms to construct the graph with 100 polygons with upto 20 points each on i7-6700k CPU. 
The path finding on the graph take negligibly small amout of time - most computations are spent on constructing the graph.
Already computed tangents are reused to check for intersections of potential edges and obstacles in O(1). A fast logarithmic method is used for checking if points are inside an obstacle and for tangents construction.

## Code Structure

```
source/          - Core library
  point.h/cpp    - 2D point/vector with dot product, cross product, length
  segment.h/cpp  - Line segment with asymmetric intersection testing
  polygon.h/cpp  - Convex polygon with tangent queries, inflation, containment
  path_finder.h/cpp - Visibility graph construction and A* pathfinding
  cone_of_vision.h/cpp - Line-of-sight/field-of-view calculations
  pointf.h       - Float comparison utilities (EPSILON, FloatEqual, FloatSign)

tests/           - Google Test unit tests
demo/            - Interactive demo application (raylib-based)
```

### Key Internal Types

The ``PathFinder`` class uses these internal data structures:
- ``vertices_`` - Vertex positions indexed by ID
- ``adjacency_list_`` - Graph edges as ``vector<vector<GraphEdge>>``
- ``point_to_vertex_id_`` - Maps coordinates to vertex IDs
- ``polygon_vertex_is_occluded_`` - Tracks vertices inside other polygons

## Demo

The demo is a cross-platform GUI application using raylib for rendering. It visualizes obstacles, the visibility graph, and computed paths.

Run the demo after building:
```bash
./build/demo/demo
```

The demo displays:
- Obstacle polygons (filled shapes)
- Visibility graph edges (optional, for debugging)
- Computed path from source to destination
- Performance timing information

## Building

### CMake (Cross-platform)

```bash
# Configure
cmake -B build

# Build
cmake --build build

# Run tests
./build/tests/tests

# Run demo
./build/demo/demo
```

Dependencies (fetched automatically via CMake FetchContent):
- raylib (demo rendering)
- Google Test (unit tests)

## Further work

The solution constructs all ``O(n)`` tangents for each point to add them as edges. Then it individually checks all edges for intersections besed on its relative position to all other tangents.
This ``O(n^2)`` part could in theory be optimized to ``O(n log n)`` by sorting all the segments from the source point by angle. Then a sweeping line algorithm should be able to find all the valid edges.
However, it couldn't dramatically improve the performance, as it's usually dominated by tangents calculations in case if most of the polygons are not intersecting (even inflated).

The interface could be updated to make it more flexible: 
Current solution run A* for each path between two points. If the map is static or very rarely updated, a Floyd-Warshall algorithm might be used instead to precalculate all possible paths.
If there's a single source and many destinations (or the other way around), a Dijkstra might be used instead of A* to get all paths in one go.

Also, the ``AddPolygons`` method could be updated to work with a timelimit and continue its work from the previous call (by splitting the loop on polygons). 
This way the caller may call this method on each frame and not loose fps. ``AddExternalPoints`` and ``GetPath`` should do nothing until ``AddPolygons`` has finished all the work.

Finally, a routine to split non-convex polygons to a unity of several convex polygons should be implemented.
