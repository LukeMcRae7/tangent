cmake_minimum_required(VERSION 3.24)
project(Tangent VERSION 0.1.0 LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE RelWithDebInfo CACHE STRING "" FORCE)
endif()

# ---- System dependencies (native Linux / Wayland) -------------------------
find_package(SDL3 REQUIRED CONFIG)

find_package(PkgConfig REQUIRED)
pkg_check_modules(EPOXY REQUIRED IMPORTED_TARGET epoxy)

# ---- Dear ImGui (docking branch) ------------------------------------------
include(FetchContent)
set(FETCHCONTENT_QUIET OFF)
FetchContent_Declare(imgui
  GIT_REPOSITORY https://github.com/ocornut/imgui.git
  GIT_TAG        fd13a1e8923a0a7077b404fc36fd063b25a0c0b5 # docking
  GIT_SHALLOW    FALSE
)
FetchContent_MakeAvailable(imgui)

add_library(imgui STATIC
  ${imgui_SOURCE_DIR}/imgui.cpp
  ${imgui_SOURCE_DIR}/imgui_draw.cpp
  ${imgui_SOURCE_DIR}/imgui_tables.cpp
  ${imgui_SOURCE_DIR}/imgui_widgets.cpp
  ${imgui_SOURCE_DIR}/imgui_demo.cpp
  ${imgui_SOURCE_DIR}/backends/imgui_impl_sdl3.cpp
  ${imgui_SOURCE_DIR}/backends/imgui_impl_opengl3.cpp
)
target_include_directories(imgui PUBLIC
  ${imgui_SOURCE_DIR} ${imgui_SOURCE_DIR}/backends)
target_link_libraries(imgui PUBLIC SDL3::SDL3)
target_compile_definitions(imgui PUBLIC IMGUI_DEFINE_MATH_OPERATORS)

# ---- Tangent ----------------------------------------------------------------
add_executable(tangent
  src/main.cpp
  src/app/application.cpp
  src/app/camera.cpp
  src/app/transform_tool.cpp
  src/app/undo.cpp
  src/mesh/halfedge.cpp
  src/mesh/primitives.cpp
  src/mesh/operations.cpp
  src/mesh/health.cpp
  src/scene/scene.cpp
  src/scene/feature.cpp
  src/render/shader.cpp
  src/render/gpu_mesh.cpp
  src/render/renderer.cpp
  src/ui/theme.cpp
  src/ui/panels.cpp
)
target_include_directories(tangent PRIVATE src)
target_link_libraries(tangent PRIVATE imgui SDL3::SDL3 PkgConfig::EPOXY)
target_compile_options(tangent PRIVATE -Wall -Wextra -Wno-unused-parameter)

# Shaders are compiled into the binary via #include of generated headers is
# overkill for now; ship them next to the executable and locate at runtime.
add_custom_command(TARGET tangent POST_BUILD
  COMMAND ${CMAKE_COMMAND} -E copy_directory
          ${CMAKE_SOURCE_DIR}/shaders $<TARGET_FILE_DIR:tangent>/shaders
  COMMENT "Staging shaders")

# ---- Tests ----------------------------------------------------------------
# Kernel-only, no GL context required, so it runs headless in CI.
enable_testing()
add_executable(test_mesh tests/test_mesh.cpp src/mesh/halfedge.cpp src/mesh/primitives.cpp)
target_include_directories(test_mesh PRIVATE src)
target_compile_options(test_mesh PRIVATE -Wall -Wextra)
add_test(NAME mesh COMMAND test_mesh)

add_executable(test_scene tests/test_scene.cpp
  src/scene/scene.cpp src/scene/feature.cpp src/mesh/operations.cpp
  src/mesh/halfedge.cpp src/mesh/primitives.cpp src/app/camera.cpp)
target_include_directories(test_scene PRIVATE src)
target_compile_options(test_scene PRIVATE -Wall -Wextra)
add_test(NAME scene COMMAND test_scene)

# Links the renderer only because TransformTool::drawOverlay references it; the
# test never creates a GL context or issues a draw.
add_executable(test_transform tests/test_transform.cpp
  src/app/transform_tool.cpp src/app/undo.cpp src/app/camera.cpp
  src/scene/scene.cpp src/scene/feature.cpp src/mesh/operations.cpp
  src/mesh/halfedge.cpp src/mesh/primitives.cpp
  src/render/renderer.cpp src/render/shader.cpp src/render/gpu_mesh.cpp)
target_include_directories(test_transform PRIVATE src)
target_link_libraries(test_transform PRIVATE PkgConfig::EPOXY)
target_compile_options(test_transform PRIVATE -Wall -Wextra -Wno-unused-parameter)
add_test(NAME transform COMMAND test_transform)

add_executable(test_operations tests/test_operations.cpp
  src/mesh/operations.cpp src/mesh/halfedge.cpp src/mesh/primitives.cpp)
target_include_directories(test_operations PRIVATE src)
target_compile_options(test_operations PRIVATE -Wall -Wextra)
add_test(NAME operations COMMAND test_operations)

add_executable(test_features tests/test_features.cpp
  src/scene/scene.cpp src/scene/feature.cpp src/mesh/operations.cpp
  src/mesh/halfedge.cpp src/mesh/primitives.cpp)
target_include_directories(test_features PRIVATE src)
target_compile_options(test_features PRIVATE -Wall -Wextra)
add_test(NAME features COMMAND test_features)

add_executable(test_health tests/test_health.cpp
  src/mesh/health.cpp src/mesh/operations.cpp src/mesh/halfedge.cpp src/mesh/primitives.cpp)
target_include_directories(test_health PRIVATE src)
target_compile_options(test_health PRIVATE -Wall -Wextra)
add_test(NAME health COMMAND test_health)
