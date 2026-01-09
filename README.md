# NavMesh

This is a small and fast library for pathfinding in 2D space around convex polygonal obstacles. This may be useful in GameDev.
The implementation can pass around the obstacles at a given distance (if the actor is not a zero-width point, but has some physical size).
The project also contains a little demo.

![Example of the demo output](https://github.com/ilyanikolaevsky/navmesh/blob/master/picture.png?raw=true)


The main feature of this library is its speed: for a moderate amount (100) of quite big polygons (20 points each) the library computes
everything in 7-8ms on a i7-6700k CPU. It doesn't require baking in the graph around the geometry, which allows completely dynamic geometry of the map.

## Usage

You need to include ``path_finder.h``, ``point.h``, ``segment.h``, ``polygon.h`` files in your project and link against the static library.
Alternatively, since it's a small project, you can just add all files from ``source/`` to your project.

The main class is ``NavMesh::PathFinder``. There are also ``NavMesh::Point``, ``NavMesh::Segment`` and ``NavMesh::Polygon`` for geometric logic.

**All coordinates use single-precision floating-point values (float).** This allows for sub-pixel precision in pathfinding calculations.

### Quick Example

```cpp
#include "path_finder.h"
#include "polygon.h"
#include "point.h"

// Create an obstacle polygon using float coordinates
NavMesh::Polygon obstacle;
obstacle.AddPoint(100.0f, 100.0f);
obstacle.AddPoint(200.0f, 100.0f);
obstacle.AddPoint(200.0f, 200.0f);
obstacle.AddPoint(100.0f, 200.0f);

// Set up pathfinder with obstacles
NavMesh::PathFinder pathfinder;
pathfinder.AddPolygons({obstacle}, 10.0f);  // 10.0f inflation distance

// Define start and end points
NavMesh::Point start(50.0f, 150.0f);
NavMesh::Point end(250.0f, 150.0f);

// Add external points and find path
pathfinder.AddExternalPoints({start, end});
std::vector<NavMesh::Point> path = pathfinder.GetPath(start, end);

// path now contains waypoints as float coordinates
for (const auto& point : path) {
    // point.x and point.y are floats
}
```

### API Reference

``PathFinder::AddPolygons`` should be called each time the map changes.
This is the slowest one, which takes ``O(n^3*k)`` time, where ``n`` is the number of polygons, and ``k`` is average number of points in each.
This method consumes ``O(n^2*k)`` memory in the worst case (usually much less, as most of the edges are not valid due to intersections).
Ideally, it should be called only if the map is updated.

``PathFinder::AddExternalPoints`` should be called after ``AddPolygons`` each time coordinates of external points change.
This method takes ``O(p*n^2)`` time and consumes ``O(n*p)`` memory, where ``p`` is the number of points added.

``PathFinder::GetPath`` should be called each time you need a path between two points. The points must be one of the external points.
This method takes ``O((n*k+p)*n*log(n*k+p))`` time and uses ``O((n+p)*n)`` memory.


## Details

This project constructs the visibility graph around obstacles using polygon sides and tangents to polygons as edges in the graph.
Then it uses A* on the constructed graph to find the shortest path. This implementation takes 8ms to construct the graph with 100 polygons with upto 20 points each on i7-6700k CPU. 
The path finding on the graph take negligibly small amout of time - most computations are spent on constructing the graph.
Already computed tangents are reused to check for intersections of potential edges and obstacles in O(1). A fast logarithmic method is used for checking if points are inside an obstacle and for tangents construction.

## Code Structure

| Directory | Contents |
|-----------|----------|
| ``source/`` | Core library files |
| ``tests/`` | GoogleTest unit tests |
| ``demo/`` | Cross-platform raylib demo application |

### Source Files

| File | Description |
|------|-------------|
| ``point.h/cpp`` | 2D point class with float coordinates (x, y), vector operations |
| ``segment.h/cpp`` | Line segment class for intersection detection |
| ``polygon.h/cpp`` | Convex polygon with float vertices, tangent computation, inflation |
| ``path_finder.h/cpp`` | Main pathfinding class - visibility graph construction and A* search |
| ``cone_of_vision.h/cpp`` | Field-of-view calculation utility |

## Demo
The demo is a cross-platform GUI application using raylib. It visualizes obstacles and paths using float coordinates, and displays performance statistics.

### Keyboard Controls

| Key | Action |
|-----|--------|
| **A** | Add polygon mode - click to add points, right-click to finish |
| **D** | Delete polygon mode - click inside polygon to delete |
| **S** | Move source point |
| **T** | Move destination/target point |
| **E** | Toggle edge visibility |
| **I** | Toggle polygon inflation (10.0f units) |
| **P** | Generate 100 random polygons |
| **C** | Generate 30 random circles (24-sided polygons) |
| **G** | Generate 10x10 grid of squares |
| **B** | Run benchmark (500 iterations) |
| **+/-** | Adjust cone of vision radius |
| **Esc** | Cancel current mode |

You can also drag the source (green) or destination (red) circles directly. The current mode is shown at the top of the window.

The upper-left corner displays timing information:
- **total**: Total time for pathfinding calculations
- **geo**: Time spent on geometry/graph construction
- **bench**: Average time from last benchmark run

Drawing is not included in measurements.

## Building

The repository has Visual Studio 2019 project files necessary to build it in ``build_vs/`` directory.
Simply open NavMesh.sln in the Visual Studio 2019 or later. ``source`` project builds a static library, ``demo`` builds a demo and ``tests`` runs tests.

On other platforms you can execute:
```
mkdir build
cd build
cmake ..
```

Then either run ``make`` or use other platform specific build tool.

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
