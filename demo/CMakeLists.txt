add_executable(
    demo
    demo.cpp
)

target_link_libraries(
    demo
    navmesh
    raylib
)

target_include_directories(demo PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../source/
)
