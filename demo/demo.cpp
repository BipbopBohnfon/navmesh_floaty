#include "point.h"
#include "pointf.h"
#include "segment.h"
#include "polygon.h"
#include "path_finder.h"
#include "cone_of_vision.h"

#include "raylib.h"

#define _USE_MATH_DEFINES
#include <math.h>

#include <chrono>
#include <vector>
#include <sstream>
#include <cstdlib>

enum class ClickMode {
    kNone,
    kAddPolygon,
    kDeletePolygon,
    kMoveSource,
    kMoveDestination
};

// Current interaction state.
ClickMode click_mode = ClickMode::kNone;

// Debug parameters, controlled from keyboard.
bool draw_edges = false;
bool inflate = true;

// Currently constructed polygon.
NavMesh::Polygon cur_polygon;

// All polygons on the map.
std::vector<NavMesh::Polygon> polygons;
bool polygons_changed = true;

// Current coordinates of source and destination nodes.
NavMesh::Point source_coordinates(30, 300);
NavMesh::Point dest_coordinates(1000, 300);

// Current cursor position.
NavMesh::Point cursor_position;

NavMesh::ConeOfVision cone_of_vision;
NavMesh::PathFinder path_finder;

// Camera for panning and zooming.
Camera2D camera = { 0 };
float camera_speed = 500.0f;  // World units per second

void GeneratePolygons() {
    const int N = 100;
    const int K = 10;
    const int world_size = 5000;  // Larger world for spatial grid benefits
    polygons.clear();
    for (int i = 0; i < N; i++) {
        NavMesh::Polygon p;
        int x = rand() % world_size;
        int y = rand() % world_size;
        for (int j = 0; j < K; ++j) {
            p.AddPoint(x + rand() % 50, y + rand() % 50);
        }
        polygons.push_back(p);
    }
    polygons_changed = true;
}

void GenerateCircles() {
    const int N = 100;
    const int K = 50;
    const int world_size = 5000;
    polygons.clear();
    for (int i = 0; i < N; i++) {
        NavMesh::Polygon p;
        int x = rand() % world_size;
        int y = rand() % world_size;
        for (int j = 0; j < K; ++j) {
            p.AddPoint(x + (int)cos(2 * M_PI * j / K) * 30, y + (int)sin(2 * M_PI * j / K) * 30);
        }
        polygons.push_back(p);
    }
    polygons_changed = true;
}

void GenerateGrid() {
    const int N = 100;
    const int grid_spacing = 200;  // Spread out over larger area
    polygons.clear();
    for (int i = 0; i < N; i++) {
        NavMesh::Polygon p;
        int x = 200 + i / 10 * grid_spacing;
        int y = 200 + i % 10 * grid_spacing;
        p.AddPoint(x + 10, y + 10);
        p.AddPoint(x - 10, y + 10);
        p.AddPoint(x - 10, y - 10);
        p.AddPoint(x + 10, y - 10);
        polygons.push_back(p);
    }
    polygons_changed = true;
}

void DrawSegment(const NavMesh::Segment& s, Color color) {
    DrawLine((int)s.b.x, (int)s.b.y, (int)s.e.x, (int)s.e.y, color);
}

void DrawPolygonOutline(const NavMesh::Polygon& p, Color color) {
    for (int i = 0; i < p.Size(); ++i) {
        DrawSegment(NavMesh::Segment(p[i], p[(i + 1) % p.Size()]), color);
    }
}

const char* GetPrompt() {
    switch (click_mode) {
    case ClickMode::kAddPolygon:
        return "LClick - add point, RClick - done";
    case ClickMode::kDeletePolygon:
        return "LClick - remove polygon, RClick - done";
    case ClickMode::kMoveSource:
        return "LClick - fix source";
    case ClickMode::kMoveDestination:
        return "LClick - fix destination";
    default:
        return "[A]dd [D]elete [S]ource [T]arget [E]dges [I]nflate [P]olygons [C]ircles [G]rid [B]ench";
    }
}

double bench_millis = 0;

void Benchmark() {
    auto start_time = std::chrono::high_resolution_clock::now();
    const int kIterations = 500;
    for (int i = 0; i < kIterations; ++i) {
        GeneratePolygons();
        path_finder.AddPolygons(polygons, inflate ? 10 : 0);
        path_finder.AddExternalPoints({ source_coordinates, dest_coordinates });
    }
    auto geo_done_time = std::chrono::high_resolution_clock::now();
    bench_millis = (geo_done_time - start_time) / std::chrono::milliseconds(1) / (double)kIterations;
}

int main() {
    const int screenWidth = 1280;
    const int screenHeight = 720;

    InitWindow(screenWidth, screenHeight, "NavMesh Demo");
    SetTargetFPS(60);

    // Initialize camera
    camera.target = (Vector2){ 500.0f, 350.0f };  // Center of original view
    camera.offset = (Vector2){ (float)screenWidth / 2.0f, (float)screenHeight / 2.0f };
    camera.rotation = 0.0f;
    camera.zoom = 1.0f;

    while (!WindowShouldClose()) {
        // Update cursor position (convert screen to world coordinates)
        Vector2 screen_pos = GetMousePosition();
        Vector2 world_pos = GetScreenToWorld2D(screen_pos, camera);
        cursor_position = NavMesh::Point(world_pos.x, world_pos.y);

        // Handle keyboard input
        if (click_mode == ClickMode::kNone) {
            if (IsKeyPressed(KEY_A)) {
                click_mode = ClickMode::kAddPolygon;
                cur_polygon.Clear();
            }
            else if (IsKeyPressed(KEY_D)) {
                click_mode = ClickMode::kDeletePolygon;
            }
            else if (IsKeyPressed(KEY_S)) {
                click_mode = ClickMode::kMoveSource;
            }
            else if (IsKeyPressed(KEY_T)) {
                click_mode = ClickMode::kMoveDestination;
            }
            else if (IsKeyPressed(KEY_E)) {
                draw_edges = !draw_edges;
            }
            else if (IsKeyPressed(KEY_I)) {
                inflate = !inflate;
                polygons_changed = true;
            }
            else if (IsKeyPressed(KEY_P)) {
                GeneratePolygons();
            }
            else if (IsKeyPressed(KEY_C)) {
                GenerateCircles();
            }
            else if (IsKeyPressed(KEY_G)) {
                GenerateGrid();
            }
            else if (IsKeyPressed(KEY_B)) {
                Benchmark();
            }
        }

        // Camera zoom with scroll wheel
        float wheel = GetMouseWheelMove();
        if (wheel != 0) {
            camera.zoom += wheel * 0.1f;
            if (camera.zoom < 0.1f) camera.zoom = 0.1f;
            if (camera.zoom > 10.0f) camera.zoom = 10.0f;
        }

        // Camera pan with arrow keys
        float dt = GetFrameTime();
        if (IsKeyDown(KEY_LEFT)) camera.target.x -= camera_speed * dt / camera.zoom;
        if (IsKeyDown(KEY_RIGHT)) camera.target.x += camera_speed * dt / camera.zoom;
        if (IsKeyDown(KEY_UP)) camera.target.y -= camera_speed * dt / camera.zoom;
        if (IsKeyDown(KEY_DOWN)) camera.target.y += camera_speed * dt / camera.zoom;

        // Handle mouse movement for dragging
        if (click_mode == ClickMode::kMoveDestination) {
            dest_coordinates = cursor_position;
        }
        else if (click_mode == ClickMode::kMoveSource) {
            source_coordinates = cursor_position;
        }

        // Handle left mouse button
        if (IsMouseButtonPressed(MOUSE_BUTTON_LEFT)) {
            if (click_mode == ClickMode::kAddPolygon) {
                cur_polygon.AddPoint(cursor_position);
            }
            else if (click_mode == ClickMode::kDeletePolygon) {
                size_t j = 0;
                for (size_t i = 0; i < polygons.size(); ++i) {
                    if (!polygons[i].IsInside(cursor_position)) {
                        polygons[j++] = std::move(polygons[i]);
                    }
                    else {
                        polygons_changed = true;
                    }
                }
                polygons.resize(j);
            }
            else if (click_mode == ClickMode::kMoveSource || click_mode == ClickMode::kMoveDestination) {
                click_mode = ClickMode::kNone;
            }
            else if (click_mode == ClickMode::kNone) {
                // Check if clicking near source or destination to start dragging
                if ((cursor_position - dest_coordinates).Len() < 10) {
                    click_mode = ClickMode::kMoveDestination;
                }
                else if ((cursor_position - source_coordinates).Len() < 10) {
                    click_mode = ClickMode::kMoveSource;
                }
            }
        }

        // Handle right mouse button
        if (IsMouseButtonPressed(MOUSE_BUTTON_RIGHT)) {
            if (click_mode == ClickMode::kAddPolygon) {
                if (cur_polygon.Size() > 2) {
                    polygons.push_back(cur_polygon);
                    polygons_changed = true;
                    cur_polygon.Clear();
                }
                else {
                    click_mode = ClickMode::kNone;
                }
            }
            else if (click_mode == ClickMode::kDeletePolygon) {
                click_mode = ClickMode::kNone;
            }
        }

        // Escape to cancel current mode
        if (IsKeyPressed(KEY_ESCAPE)) {
            click_mode = ClickMode::kNone;
            cur_polygon.Clear();
        }

        // Update pathfinder
        auto start_time = std::chrono::high_resolution_clock::now();

        if (polygons_changed) {
            path_finder.AddPolygons(polygons, inflate ? 10 : 0);
            cone_of_vision.AddPolygons(polygons);
            polygons_changed = false;
        }
        path_finder.AddExternalPoints({ source_coordinates, dest_coordinates });

        auto geo_done_time = std::chrono::high_resolution_clock::now();

        std::vector<NavMesh::Point> path = path_finder.GetPath(source_coordinates, dest_coordinates);
        std::vector<NavMesh::PointF> vision = cone_of_vision.GetVision(source_coordinates, 100);

        auto end_time = std::chrono::high_resolution_clock::now();

        // Drawing
        BeginDrawing();
        ClearBackground(WHITE);

        // Begin camera mode for world-space drawing
        BeginMode2D(camera);

        // Draw all polygons
        for (const auto& p : polygons) {
            bool red = (click_mode == ClickMode::kDeletePolygon) && p.IsInside(cursor_position);
            DrawPolygonOutline(p, red ? RED : BLACK);
        }

        // Draw currently being constructed polygon
        if (click_mode == ClickMode::kAddPolygon) {
            if (cur_polygon.Size() > 0) {
                DrawPolygonOutline(cur_polygon, GRAY);
            }

            if (cur_polygon.Size() > 0 && !cur_polygon.IsInside(cursor_position)) {
                auto tangents = cur_polygon.GetTangentIds(cursor_position);
                if (tangents.first >= 0 && tangents.second >= 0) {
                    DrawSegment(NavMesh::Segment(cursor_position, cur_polygon[tangents.first]), RED);
                    DrawSegment(NavMesh::Segment(cursor_position, cur_polygon[tangents.second]), GREEN);
                }
            }
        }

        // Draw debug edges
        if (draw_edges) {
            auto edges = path_finder.GetEdgesForDebug();
            for (const auto& e : edges) {
                DrawSegment(e, GRAY);
            }
        }

        // Draw the path
        for (size_t i = 0; i + 1 < path.size(); ++i) {
            DrawSegment(NavMesh::Segment(path[i], path[i + 1]), RED);
        }

        // Draw cone of vision
        for (size_t i = 0; i + 1 < vision.size(); ++i) {
            DrawSegment(NavMesh::Segment((NavMesh::Point)vision[i], (NavMesh::Point)vision[i + 1]), BLACK);
        }

        // Draw source and destination as circles
        DrawCircleLines((int)source_coordinates.x, (int)source_coordinates.y, 5, GREEN);
        DrawCircleLines((int)dest_coordinates.x, (int)dest_coordinates.y, 5, RED);

        // End camera mode before drawing UI
        EndMode2D();

        // Draw UI elements (screen-space, not affected by camera)
        DrawText(GetPrompt(), 400, 20, 16, BLACK);

        // Draw timing info
        int64_t millis_total = (end_time - start_time) / std::chrono::milliseconds(1);
        int64_t millis_graph = (geo_done_time - start_time) / std::chrono::milliseconds(1);
        std::stringstream ss;
        ss << "total:" << millis_total << "ms  geo:" << millis_graph << "ms  bench:" << bench_millis << "ms";
        DrawText(ss.str().c_str(), 10, 10, 16, BLACK);

        // Draw toggle states
        std::stringstream toggles;
        toggles << "Edges: " << (draw_edges ? "ON" : "OFF") << "  Inflate: " << (inflate ? "ON" : "OFF");
        DrawText(toggles.str().c_str(), 10, 30, 14, DARKGRAY);

        // Draw camera state
        std::stringstream cam_info;
        cam_info << "Zoom: " << camera.zoom << "  Target: (" << (int)camera.target.x << ", " << (int)camera.target.y << ")";
        DrawText(cam_info.str().c_str(), 10, 50, 14, DARKGRAY);

        // Draw edge count if enabled
        if (draw_edges) {
            auto edges = path_finder.GetEdgesForDebug();
            std::stringstream edge_ss;
            edge_ss << edges.size() << " edges";
            DrawText(edge_ss.str().c_str(), 10, 70, 14, DARKGRAY);
        }

        // Draw controls hint
        DrawText("Arrow keys: Pan | Scroll: Zoom", 10, screenHeight - 20, 14, DARKGRAY);

        EndDrawing();
    }

    CloseWindow();
    return 0;
}
