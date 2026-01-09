#include "point.h"
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
#include <iomanip>

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
int cone_radius = 100;

// Currently constructed polygon.
NavMesh::Polygon cur_polygon;

// All polygons on the map.
std::vector<NavMesh::Polygon> polygons;
bool polygons_changed = true;

// Current coordinates of source and destination nodes.
NavMesh::Point source_coordinates(30.0f, 300.0f);
NavMesh::Point dest_coordinates(1000.0f, 300.0f);

// Current cursor position (in world coordinates).
NavMesh::Point cursor_position;

NavMesh::ConeOfVision cone_of_vision;
NavMesh::PathFinder path_finder;

// Camera for zoom/pan
Camera2D camera = { 0 };
float camera_speed = 500.0f;

// Scalability test results
struct ScalabilityResult {
    int num_polygons;
    int num_vertices;
    double add_polygons_ms;
    double add_external_ms;
    double get_path_ms;
    size_t edge_count;
};
std::vector<ScalabilityResult> scalability_results;
bool scalability_test_running = false;
bool scalability_test_complete = false;
std::string scalability_status = "";

void GeneratePolygons() {
    const int N = 100;
    const int K = 10;
    polygons.clear();
    for (int i = 0; i < N; i++) {
        NavMesh::Polygon p;
        float x = 30.0f + static_cast<float>(rand() % 1000);
        float y = 30.0f + static_cast<float>(rand() % 600);
        for (int j = 0; j < K; ++j) {
            p.AddPoint(x + static_cast<float>(rand() % 50), y + static_cast<float>(rand() % 50));
        }
        polygons.push_back(p);
    }
    polygons_changed = true;
}

void GenerateCircles() {
    const int N = 30;
    const int K = 24;
    const float R = 25.0f;
    polygons.clear();
    for (int i = 0; i < N; i++) {
        NavMesh::Polygon p;
        float x = R + static_cast<float>(rand() % 900);
        float y = R + static_cast<float>(rand() % 500);
        for (int j = 0; j < K; ++j) {
            float angle = 2.0f * static_cast<float>(M_PI) * static_cast<float>(j) / static_cast<float>(K);
            p.AddPoint(x + cosf(angle) * R, y + sinf(angle) * R);
        }
        polygons.push_back(p);
    }
    polygons_changed = true;
}

void GenerateGrid() {
    const int N = 100;
    polygons.clear();
    for (int i = 0; i < N; i++) {
        NavMesh::Polygon p;
        float x = 200.0f + static_cast<float>(i / 10) * 40.0f;
        float y = 200.0f + static_cast<float>(i % 10) * 40.0f;
        p.AddPoint(x + 10.0f, y + 10.0f);
        p.AddPoint(x - 10.0f, y + 10.0f);
        p.AddPoint(x - 10.0f, y - 10.0f);
        p.AddPoint(x + 10.0f, y - 10.0f);
        polygons.push_back(p);
    }
    polygons_changed = true;
}

// Generate a grid of rectangles for scalability testing
void GenerateScalabilityGrid(int count, float world_size) {
    polygons.clear();
    polygons.reserve(count);

    float cell_size = world_size / sqrtf(static_cast<float>(count));
    int grid_side = static_cast<int>(sqrtf(static_cast<float>(count))) + 1;
    float rect_size = cell_size * 0.6f;

    for (int i = 0; i < count; i++) {
        float base_x = static_cast<float>(i % grid_side) * cell_size + cell_size * 0.2f;
        float base_y = static_cast<float>(i / grid_side) * cell_size + cell_size * 0.2f;

        NavMesh::Polygon p;
        p.AddPoint(base_x, base_y);
        p.AddPoint(base_x + rect_size, base_y);
        p.AddPoint(base_x + rect_size, base_y + rect_size);
        p.AddPoint(base_x, base_y + rect_size);
        polygons.push_back(p);
    }
    polygons_changed = true;
}

ScalabilityResult RunSingleScalabilityTest(int num_polygons, float world_size, float inflate_by) {
    ScalabilityResult result;
    result.num_polygons = num_polygons;

    // Generate polygons
    GenerateScalabilityGrid(num_polygons, world_size);
    result.num_vertices = num_polygons * 4;
    if (inflate_by > 0) result.num_vertices = num_polygons * 8;

    NavMesh::PathFinder pf;

    // Benchmark AddPolygons
    auto start = std::chrono::high_resolution_clock::now();
    pf.AddPolygons(polygons, inflate_by);
    auto end = std::chrono::high_resolution_clock::now();
    result.add_polygons_ms = std::chrono::duration<double, std::milli>(end - start).count();

    // Define start/end points
    NavMesh::Point source(0.0f, 0.0f);
    NavMesh::Point dest(world_size - 1.0f, world_size - 1.0f);

    // Benchmark AddExternalPoints
    start = std::chrono::high_resolution_clock::now();
    pf.AddExternalPoints({source, dest});
    end = std::chrono::high_resolution_clock::now();
    result.add_external_ms = std::chrono::duration<double, std::milli>(end - start).count();

    // Count edges
    auto edges = pf.GetEdgesForDebug();
    result.edge_count = edges.size();

    // Benchmark GetPath
    start = std::chrono::high_resolution_clock::now();
    auto path = pf.GetPath(source, dest);
    end = std::chrono::high_resolution_clock::now();
    result.get_path_ms = std::chrono::duration<double, std::milli>(end - start).count();

    return result;
}

void RunScalabilityTest() {
    scalability_results.clear();
    scalability_test_running = true;
    scalability_test_complete = false;
    scalability_status = "Running scalability test (with spatial optimization)...";

    // Test larger sizes now that we have spatial optimization
    std::vector<int> test_sizes = {100, 500, 1000, 2000, 5000, 7500, 10000, 15000, 20000, 30000, 50000};

    for (int n : test_sizes) {
        float world_size = sqrtf(static_cast<float>(n)) * 100.0f;

        std::stringstream ss;
        ss << "Testing n=" << n << " polygons...";
        scalability_status = ss.str();

        auto result = RunSingleScalabilityTest(n, world_size, 10.0f);
        scalability_results.push_back(result);

        // Stop if taking too long (> 60 seconds for AddPolygons)
        if (result.add_polygons_ms > 60000) {
            scalability_status = "Stopped: exceeded 60 second threshold";
            break;
        }
    }

    scalability_test_running = false;
    scalability_test_complete = true;

    // Restore normal view
    GeneratePolygons();
}

void DrawSegmentWorld(const NavMesh::Segment& s, Color color) {
    DrawLine(static_cast<int>(s.b.x), static_cast<int>(s.b.y),
             static_cast<int>(s.e.x), static_cast<int>(s.e.y), color);
}

void DrawPolygonOutline(const NavMesh::Polygon& p, Color color) {
    for (int i = 0; i < p.Size(); ++i) {
        DrawSegmentWorld(NavMesh::Segment(p[i], p[(i + 1) % p.Size()]), color);
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
        return "[A]dd [D]el [S]rc [T]gt [E]dges [I]nfl [P]oly [C]ircle [G]rid [B]ench [K]scale | Arrows:pan Scroll:zoom";
    }
}

double bench_millis = 0;

void Benchmark() {
    auto start_time = std::chrono::high_resolution_clock::now();
    const int kIterations = 500;
    for (int i = 0; i < kIterations; ++i) {
        GeneratePolygons();
        path_finder.AddPolygons(polygons, inflate ? 10.0f : 0.0f);
        path_finder.AddExternalPoints({ source_coordinates, dest_coordinates });
    }
    auto geo_done_time = std::chrono::high_resolution_clock::now();
    bench_millis = (geo_done_time - start_time) / std::chrono::milliseconds(1) / (double)kIterations;
}

int main() {
    const int screenWidth = 1280;
    const int screenHeight = 720;

    InitWindow(screenWidth, screenHeight, "NavMesh Demo - Scalability Test");
    SetTargetFPS(60);

    // Initialize camera
    camera.target = (Vector2){ 640.0f, 360.0f };
    camera.offset = (Vector2){ screenWidth / 2.0f, screenHeight / 2.0f };
    camera.rotation = 0.0f;
    camera.zoom = 1.0f;

    while (!WindowShouldClose()) {
        float dt = GetFrameTime();

        // Camera controls - arrow keys for panning
        if (IsKeyDown(KEY_RIGHT)) camera.target.x += camera_speed * dt / camera.zoom;
        if (IsKeyDown(KEY_LEFT)) camera.target.x -= camera_speed * dt / camera.zoom;
        if (IsKeyDown(KEY_DOWN)) camera.target.y += camera_speed * dt / camera.zoom;
        if (IsKeyDown(KEY_UP)) camera.target.y -= camera_speed * dt / camera.zoom;

        // Camera zoom with scroll wheel
        float wheel = GetMouseWheelMove();
        if (wheel != 0) {
            // Get world point before zoom
            Vector2 mouseWorldPos = GetScreenToWorld2D(GetMousePosition(), camera);

            // Zoom
            camera.zoom += wheel * 0.1f * camera.zoom;
            if (camera.zoom < 0.01f) camera.zoom = 0.01f;
            if (camera.zoom > 50.0f) camera.zoom = 50.0f;

            // Get world point after zoom and adjust offset to keep mouse position stable
            Vector2 mouseWorldPosAfter = GetScreenToWorld2D(GetMousePosition(), camera);
            camera.target.x += mouseWorldPos.x - mouseWorldPosAfter.x;
            camera.target.y += mouseWorldPos.y - mouseWorldPosAfter.y;
        }

        // Reset camera with Home key
        if (IsKeyPressed(KEY_HOME)) {
            camera.target = (Vector2){ 640.0f, 360.0f };
            camera.zoom = 1.0f;
        }

        // Update cursor position in world coordinates
        Vector2 mouseWorld = GetScreenToWorld2D(GetMousePosition(), camera);
        cursor_position = NavMesh::Point(mouseWorld.x, mouseWorld.y);

        // Handle keyboard input
        if (click_mode == ClickMode::kNone && !scalability_test_running) {
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
            else if (IsKeyPressed(KEY_K)) {
                RunScalabilityTest();
            }
        }

        // Cone radius controls (work in any mode)
        if (IsKeyPressed(KEY_EQUAL) || IsKeyPressed(KEY_KP_ADD)) {
            cone_radius += 10;
        }
        if (IsKeyPressed(KEY_MINUS) || IsKeyPressed(KEY_KP_SUBTRACT)) {
            cone_radius = (cone_radius > 10) ? cone_radius - 10 : 10;
        }

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
                if ((cursor_position - dest_coordinates).Len() < 10.0f / camera.zoom) {
                    click_mode = ClickMode::kMoveDestination;
                }
                else if ((cursor_position - source_coordinates).Len() < 10.0f / camera.zoom) {
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
            scalability_test_complete = false; // Clear results display
        }

        // Update pathfinder
        auto start_time = std::chrono::high_resolution_clock::now();

        if (polygons_changed) {
            path_finder.AddPolygons(polygons, inflate ? 10.0f : 0.0f);
            cone_of_vision.AddPolygons(polygons);
            polygons_changed = false;
        }
        path_finder.AddExternalPoints({ source_coordinates, dest_coordinates });

        auto geo_done_time = std::chrono::high_resolution_clock::now();

        std::vector<NavMesh::Point> path = path_finder.GetPath(source_coordinates, dest_coordinates);
        std::vector<NavMesh::Point> vision = cone_of_vision.GetVision(source_coordinates, cone_radius);

        auto end_time = std::chrono::high_resolution_clock::now();

        // Drawing
        BeginDrawing();
        ClearBackground(WHITE);

        // Begin 2D camera mode for world-space drawing
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
                    DrawSegmentWorld(NavMesh::Segment(cursor_position, cur_polygon[tangents.first]), RED);
                    DrawSegmentWorld(NavMesh::Segment(cursor_position, cur_polygon[tangents.second]), GREEN);
                }
            }
        }

        // Draw debug edges
        if (draw_edges) {
            auto edges = path_finder.GetEdgesForDebug();
            for (const auto& e : edges) {
                DrawSegmentWorld(e, GRAY);
            }
        }

        // Draw the path
        for (size_t i = 0; i + 1 < path.size(); ++i) {
            DrawSegmentWorld(NavMesh::Segment(path[i], path[i + 1]), RED);
        }

        // Draw cone of vision
        for (size_t i = 0; i + 1 < vision.size(); ++i) {
            DrawSegmentWorld(NavMesh::Segment(vision[i], vision[i + 1]), BLACK);
        }

        // Draw source and destination as circles
        DrawCircleLines(static_cast<int>(source_coordinates.x), static_cast<int>(source_coordinates.y), 5.0f, GREEN);
        DrawCircleLines(static_cast<int>(dest_coordinates.x), static_cast<int>(dest_coordinates.y), 5.0f, RED);

        EndMode2D();

        // UI Drawing (screen space - not affected by camera)

        // Draw prompt
        DrawText(GetPrompt(), 10, 50, 14, BLACK);

        // Draw timing info
        int64_t millis_total = (end_time - start_time) / std::chrono::milliseconds(1);
        int64_t millis_graph = (geo_done_time - start_time) / std::chrono::milliseconds(1);
        std::stringstream ss;
        ss << "total:" << millis_total << "ms  geo:" << millis_graph << "ms  bench:" << bench_millis << "ms";
        DrawText(ss.str().c_str(), 10, 10, 16, BLACK);

        // Draw toggle states and camera info
        std::stringstream toggles;
        toggles << "Edges:" << (draw_edges ? "ON" : "OFF")
                << " Inflate:" << (inflate ? "ON" : "OFF")
                << " Zoom:" << std::fixed << std::setprecision(2) << camera.zoom
                << " Polys:" << polygons.size();
        DrawText(toggles.str().c_str(), 10, 30, 14, DARKGRAY);

        // Draw edge count if edges visible
        if (draw_edges) {
            auto edges = path_finder.GetEdgesForDebug();
            std::stringstream es;
            es << edges.size() << " edges";
            DrawText(es.str().c_str(), 1100, 10, 16, BLACK);
        }

        // Draw scalability test results
        if (scalability_test_complete && !scalability_results.empty()) {
            DrawRectangle(10, 80, 500, 30 + static_cast<int>(scalability_results.size()) * 20 + 100, Fade(WHITE, 0.9f));
            DrawRectangleLines(10, 80, 500, 30 + static_cast<int>(scalability_results.size()) * 20 + 100, BLACK);

            DrawText("SCALABILITY TEST RESULTS (Press ESC to close)", 20, 90, 14, DARKBLUE);
            DrawText("Polygons  Vertices  AddPolygons   AddExternal   GetPath    Edges", 20, 115, 12, BLACK);

            int y = 135;
            for (const auto& r : scalability_results) {
                std::stringstream line;
                line << std::setw(8) << r.num_polygons
                     << std::setw(10) << r.num_vertices
                     << std::setw(11) << std::fixed << std::setprecision(1) << r.add_polygons_ms << "ms"
                     << std::setw(11) << std::fixed << std::setprecision(2) << r.add_external_ms << "ms"
                     << std::setw(10) << std::fixed << std::setprecision(2) << r.get_path_ms << "ms"
                     << std::setw(10) << r.edge_count;
                DrawText(line.str().c_str(), 20, y, 12, BLACK);
                y += 18;
            }

            // Show analysis
            if (scalability_results.size() >= 2) {
                y += 10;
                DrawText("--- Performance Analysis (with Spatial Grid) ---", 20, y, 12, DARKBLUE);
                y += 18;

                auto& last = scalability_results.back();

                // Check if we reached 50K
                if (last.num_polygons >= 50000) {
                    std::stringstream msg;
                    msg << "50,000 polygons completed in " << std::fixed << std::setprecision(1)
                        << last.add_polygons_ms / 1000.0 << " seconds!";
                    DrawText(msg.str().c_str(), 20, y, 12, GREEN);
                    y += 18;
                    DrawText("Spatial grid optimization: SUCCESS", 20, y, 12, GREEN);
                } else {
                    // Estimate based on observed scaling
                    auto& r1 = scalability_results[scalability_results.size() - 2];
                    auto& r2 = last;

                    double n1 = r1.num_polygons;
                    double n2 = r2.num_polygons;
                    double t1 = r1.add_polygons_ms;
                    double t2 = r2.add_polygons_ms;

                    // Estimate scaling exponent: t = C * n^a, so a = log(t2/t1) / log(n2/n1)
                    double scaling_exp = std::log(t2 / t1) / std::log(n2 / n1);

                    std::stringstream exp_str;
                    exp_str << "Observed scaling: O(n^" << std::fixed << std::setprecision(2) << scaling_exp << ")";
                    DrawText(exp_str.str().c_str(), 20, y, 12, (scaling_exp < 2.5) ? GREEN : ORANGE);
                    y += 18;

                    // Extrapolate to 50K
                    double c = t2 / std::pow(n2, scaling_exp);
                    double estimated_50k_ms = c * std::pow(50000.0, scaling_exp);

                    std::stringstream est;
                    if (estimated_50k_ms > 3600000) {
                        est << "Estimated 50K time: ~" << std::fixed << std::setprecision(1) << estimated_50k_ms / 3600000.0 << " hours";
                    } else if (estimated_50k_ms > 60000) {
                        est << "Estimated 50K time: ~" << std::fixed << std::setprecision(1) << estimated_50k_ms / 60000.0 << " minutes";
                    } else {
                        est << "Estimated 50K time: ~" << std::fixed << std::setprecision(1) << estimated_50k_ms / 1000.0 << " seconds";
                    }
                    DrawText(est.str().c_str(), 20, y, 12, (estimated_50k_ms < 300000) ? GREEN : ORANGE);
                }
                y += 18;
                DrawText("Optimizations: Spatial Grid + AABB filtering", 20, y, 12, DARKGRAY);
            }
        }

        if (scalability_test_running) {
            DrawRectangle(screenWidth/2 - 150, screenHeight/2 - 30, 300, 60, Fade(WHITE, 0.95f));
            DrawRectangleLines(screenWidth/2 - 150, screenHeight/2 - 30, 300, 60, BLACK);
            DrawText(scalability_status.c_str(), screenWidth/2 - 140, screenHeight/2 - 10, 16, BLACK);
        }

        EndDrawing();
    }

    CloseWindow();
    return 0;
}
