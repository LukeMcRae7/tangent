cmake_minimum_required(VERSION 3.24)

# Declared before project() because on Windows the vcpkg toolchain resolves the
# manifest while project() runs, and has to know by then whether OpenCASCADE is
# wanted. The option and vcpkg.json's "brep" feature are the same switch, so
# -DTANGENT_BREP=OFF means the same thing on both platforms.
option(TANGENT_BREP "Build the exact B-rep backend (requires OpenCASCADE)" ON)
if(TANGENT_BREP)
  list(APPEND VCPKG_MANIFEST_FEATURES brep)
endif()

project(Tangent VERSION 0.1.0 LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE RelWithDebInfo CACHE STRING "" FORCE)
endif()

# ---- System dependencies --------------------------------------------------
# Linux takes these from the system (pacman, apt); Windows takes them from
# vcpkg via the manifest next to this file. SDL3 ships a CMake config either
# way, so it needs no help.
find_package(SDL3 REQUIRED CONFIG)

# libepoxy is a meson project. Linux exposes it through pkg-config; the vcpkg
# build ships neither a CMake config nor a usable .pc, so there it is found by
# hand. Both arrive as one target and nothing below this needs to know which
# platform it is on.
add_library(tangent_epoxy INTERFACE)
if(WIN32)
  find_path(EPOXY_INCLUDE_DIR NAMES epoxy/gl.h REQUIRED)
  find_library(EPOXY_LIBRARY NAMES epoxy REQUIRED)
  message(STATUS "epoxy ${EPOXY_LIBRARY}")
  target_include_directories(tangent_epoxy INTERFACE ${EPOXY_INCLUDE_DIR})
  target_link_libraries(tangent_epoxy INTERFACE ${EPOXY_LIBRARY})
else()
  find_package(PkgConfig REQUIRED)
  pkg_check_modules(EPOXY REQUIRED IMPORTED_TARGET epoxy)
  target_link_libraries(tangent_epoxy INTERFACE PkgConfig::EPOXY)
endif()

# Warning flags, spelled per compiler: MSVC understands neither -Wextra nor
# -Wno-unused-parameter (4100 is its number for the latter).
if(MSVC)
  set(TANGENT_WARNINGS /W4)
  set(TANGENT_WARNINGS_LOOSE /W4 /wd4100)
  # Defensive against <windows.h>, which epoxy's WGL path can pull in: NOMINMAX
  # stops it taking min/max, and WIN32_LEAN_AND_MEAN trims what else it brings.
  add_compile_definitions(NOMINMAX WIN32_LEAN_AND_MEAN _USE_MATH_DEFINES
                          _CRT_SECURE_NO_WARNINGS)
else()
  set(TANGENT_WARNINGS -Wall -Wextra)
  set(TANGENT_WARNINGS_LOOSE -Wall -Wextra -Wno-unused-parameter)
endif()

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
# ---- Optional: the exact B-rep backend ------------------------------------
#
# Off by default, and the build without it is exactly the build that shipped
# before: the same sources, the same behaviour, no new dependency. With it on,
# src/geom gains a second backend behind the same Body interface.
#
# The definition and the libraries are set for the whole directory rather than
# per target on purpose. Body's layout depends on TG_HAVE_OCCT, so every
# translation unit in the program has to agree about it -- one target compiled
# without it would be a one-definition-rule violation that shows up as a crash
# rather than as an error.
# The option itself is declared at the top of this file, before project(),
# because vcpkg needs to know about it while the manifest is being resolved.

set(TANGENT_BREP_SRC src/geom/brep_stub.cpp)
if(TANGENT_BREP)
  # Required, and said plainly. Falling back to the mesh kernel when the
  # dependency is missing would hand someone a build that quietly is not the
  # one they think they have -- and "why is my hole a polygon" is a bad way to
  # find out.
  find_package(OpenCASCADE CONFIG)
  if(NOT OpenCASCADE_FOUND)
    message(FATAL_ERROR
      "OpenCASCADE was not found, and it is what makes a hole round.\n"
      "  Install it (Arch: pacman -S opencascade, Debian: libocct-*-dev),\n"
      "  or on Windows configure with the vcpkg toolchain, which takes it\n"
      "  from the manifest (see CMakePresets.json),\n"
      "  or point CMake at a prefix with -DCMAKE_PREFIX_PATH=<prefix>,\n"
      "  or build the mesh kernel alone with -DTANGENT_BREP=OFF.")
  endif()
  message(STATUS "OpenCASCADE ${OpenCASCADE_VERSION} from ${OpenCASCADE_INSTALL_PREFIX}")
  set(TANGENT_BREP_SRC src/geom/brep_occt.cpp)
  add_compile_definitions(TG_HAVE_OCCT)
  include_directories(SYSTEM ${OpenCASCADE_INCLUDE_DIR})
  # Modelling only. Visualization is what would drag in vtk, tcl/tk, freetype
  # and X; Tangent has its own renderer and needs none of it.
  link_libraries(
    ${OpenCASCADE_FoundationClasses_LIBRARIES}
    ${OpenCASCADE_ModelingData_LIBRARIES}
    ${OpenCASCADE_ModelingAlgorithms_LIBRARIES})
endif()

set(TANGENT_GEOM src/geom/body.cpp src/geom/operations.cpp
                 src/geom/kernel_guard.cpp ${TANGENT_BREP_SRC})

add_executable(tangent
  src/main.cpp
  src/app/application.cpp
  src/app/camera.cpp
  src/app/transform_tool.cpp
  src/app/create_tool.cpp
  src/app/measure.cpp
  src/app/snap.cpp
  src/app/drag_axis.cpp
  src/app/undo.cpp
  src/mesh/halfedge.cpp
  src/mesh/primitives.cpp
  ${TANGENT_GEOM} src/mesh/operations.cpp
  src/mesh/health.cpp
  src/mesh/boolean.cpp
  src/mesh/export_stl.cpp
  src/scene/scene.cpp
  src/scene/feature.cpp
  src/scene/serialize.cpp
  src/render/shader.cpp
  src/render/gpu_mesh.cpp
  src/render/renderer.cpp
  src/render/lod.cpp
  src/ui/theme.cpp
  src/ui/panels.cpp
)
target_include_directories(tangent PRIVATE src)
target_link_libraries(tangent PRIVATE imgui SDL3::SDL3 tangent_epoxy)
target_compile_options(tangent PRIVATE ${TANGENT_WARNINGS_LOOSE})

# Shaders are compiled into the binary via #include of generated headers is
# overkill for now; ship them next to the executable and locate at runtime.
add_custom_command(TARGET tangent POST_BUILD
  COMMAND ${CMAKE_COMMAND} -E copy_directory
          ${CMAKE_SOURCE_DIR}/shaders $<TARGET_FILE_DIR:tangent>/shaders
  COMMENT "Staging shaders")

# ---- Tests ----------------------------------------------------------------
# Kernel-only, no GL context required, so it runs headless in CI.
enable_testing()
# The seam. These are the tests a second geometry backend must pass unchanged.
add_executable(test_body tests/test_body.cpp
  ${TANGENT_GEOM}
  src/mesh/boolean.cpp src/mesh/health.cpp src/mesh/operations.cpp
  src/mesh/halfedge.cpp src/mesh/primitives.cpp)
target_include_directories(test_body PRIVATE src)
target_compile_options(test_body PRIVATE ${TANGENT_WARNINGS})
add_test(NAME body COMMAND test_body)

add_executable(test_mesh tests/test_mesh.cpp src/mesh/halfedge.cpp src/mesh/primitives.cpp)
target_include_directories(test_mesh PRIVATE src)
target_compile_options(test_mesh PRIVATE ${TANGENT_WARNINGS})
add_test(NAME mesh COMMAND test_mesh)

add_executable(test_scene tests/test_scene.cpp
  src/scene/scene.cpp src/scene/feature.cpp src/mesh/boolean.cpp src/mesh/health.cpp ${TANGENT_GEOM} src/mesh/operations.cpp
  src/mesh/halfedge.cpp src/mesh/primitives.cpp src/app/camera.cpp)
target_include_directories(test_scene PRIVATE src)
target_compile_options(test_scene PRIVATE ${TANGENT_WARNINGS})
add_test(NAME scene COMMAND test_scene)

# Links the renderer only because TransformTool::drawOverlay references it; the
# test never creates a GL context or issues a draw.
add_executable(test_transform tests/test_transform.cpp
  src/app/transform_tool.cpp src/app/undo.cpp src/app/camera.cpp
  src/scene/scene.cpp src/scene/feature.cpp src/mesh/boolean.cpp src/mesh/health.cpp ${TANGENT_GEOM} src/mesh/operations.cpp
  src/mesh/halfedge.cpp src/mesh/primitives.cpp
  src/render/renderer.cpp src/render/shader.cpp src/render/gpu_mesh.cpp)
target_include_directories(test_transform PRIVATE src)
target_link_libraries(test_transform PRIVATE tangent_epoxy)
target_compile_options(test_transform PRIVATE ${TANGENT_WARNINGS_LOOSE})
add_test(NAME transform COMMAND test_transform)

add_executable(test_operations tests/test_operations.cpp
  src/mesh/boolean.cpp src/mesh/health.cpp ${TANGENT_GEOM} src/mesh/operations.cpp src/mesh/halfedge.cpp src/mesh/primitives.cpp)
target_include_directories(test_operations PRIVATE src)
target_compile_options(test_operations PRIVATE ${TANGENT_WARNINGS})
add_test(NAME operations COMMAND test_operations)

add_executable(test_fillet tests/test_fillet.cpp
  src/mesh/health.cpp src/mesh/boolean.cpp ${TANGENT_GEOM} src/mesh/operations.cpp src/mesh/halfedge.cpp src/mesh/primitives.cpp)
target_include_directories(test_fillet PRIVATE src)
target_compile_options(test_fillet PRIVATE ${TANGENT_WARNINGS})
add_test(NAME fillet COMMAND test_fillet)

add_executable(test_names tests/test_names.cpp
  src/mesh/health.cpp src/mesh/boolean.cpp ${TANGENT_GEOM} src/mesh/operations.cpp src/mesh/halfedge.cpp src/mesh/primitives.cpp)
target_include_directories(test_names PRIVATE src)
target_compile_options(test_names PRIVATE ${TANGENT_WARNINGS})
add_test(NAME names COMMAND test_names)

add_executable(test_fillet_mechanics tests/test_fillet_mechanics.cpp
  src/mesh/health.cpp src/mesh/boolean.cpp ${TANGENT_GEOM} src/mesh/operations.cpp
  src/mesh/halfedge.cpp src/mesh/primitives.cpp)
target_include_directories(test_fillet_mechanics PRIVATE src)
target_compile_options(test_fillet_mechanics PRIVATE ${TANGENT_WARNINGS})
add_test(NAME fillet_mechanics COMMAND test_fillet_mechanics)

add_executable(test_fillet_sweep tests/test_fillet_sweep.cpp
  src/mesh/health.cpp src/mesh/boolean.cpp ${TANGENT_GEOM} src/mesh/operations.cpp src/mesh/halfedge.cpp src/mesh/primitives.cpp)
target_include_directories(test_fillet_sweep PRIVATE src)
target_compile_options(test_fillet_sweep PRIVATE ${TANGENT_WARNINGS})
add_test(NAME fillet_sweep COMMAND test_fillet_sweep)

add_executable(test_features tests/test_features.cpp
  src/scene/scene.cpp src/scene/feature.cpp src/mesh/boolean.cpp src/mesh/health.cpp
  ${TANGENT_GEOM} src/mesh/operations.cpp src/mesh/halfedge.cpp src/mesh/primitives.cpp)
target_include_directories(test_features PRIVATE src)
target_compile_options(test_features PRIVATE ${TANGENT_WARNINGS})
add_test(NAME features COMMAND test_features)

add_executable(test_health tests/test_health.cpp
  src/mesh/health.cpp src/mesh/boolean.cpp ${TANGENT_GEOM} src/mesh/operations.cpp src/mesh/halfedge.cpp src/mesh/primitives.cpp)
target_include_directories(test_health PRIVATE src)
target_compile_options(test_health PRIVATE ${TANGENT_WARNINGS})
add_test(NAME health COMMAND test_health)

add_executable(test_boolean tests/test_boolean.cpp
  src/mesh/boolean.cpp src/mesh/health.cpp ${TANGENT_GEOM} src/mesh/operations.cpp
  src/mesh/halfedge.cpp src/mesh/primitives.cpp)
target_include_directories(test_boolean PRIVATE src)
target_compile_options(test_boolean PRIVATE ${TANGENT_WARNINGS})
add_test(NAME boolean COMMAND test_boolean)

# Links the renderer only because MeasureTool::drawOverlay references it; no GL
# context is created.
add_executable(test_measure tests/test_measure.cpp
  src/app/measure.cpp src/app/camera.cpp src/scene/scene.cpp src/scene/feature.cpp src/mesh/boolean.cpp
  src/mesh/health.cpp ${TANGENT_GEOM} src/mesh/operations.cpp src/mesh/halfedge.cpp src/mesh/primitives.cpp
  src/render/renderer.cpp src/render/shader.cpp src/render/gpu_mesh.cpp)
target_include_directories(test_measure PRIVATE src)
target_link_libraries(test_measure PRIVATE tangent_epoxy)
target_compile_options(test_measure PRIVATE ${TANGENT_WARNINGS_LOOSE})
add_test(NAME measure COMMAND test_measure)

add_executable(test_io tests/test_io.cpp
  src/mesh/export_stl.cpp src/scene/serialize.cpp
  src/scene/scene.cpp src/scene/feature.cpp src/mesh/boolean.cpp
  src/mesh/health.cpp ${TANGENT_GEOM} src/mesh/operations.cpp src/mesh/halfedge.cpp src/mesh/primitives.cpp)
target_include_directories(test_io PRIVATE src)
target_compile_options(test_io PRIVATE ${TANGENT_WARNINGS})
add_test(NAME io COMMAND test_io)

add_executable(test_create_tool tests/test_create_tool.cpp
  src/app/create_tool.cpp src/app/camera.cpp src/app/undo.cpp src/app/snap.cpp
  src/scene/scene.cpp src/scene/feature.cpp src/mesh/boolean.cpp
  src/mesh/health.cpp ${TANGENT_GEOM} src/mesh/operations.cpp src/mesh/halfedge.cpp src/mesh/primitives.cpp
  src/render/renderer.cpp src/render/shader.cpp src/render/gpu_mesh.cpp)
target_include_directories(test_create_tool PRIVATE src ${imgui_SOURCE_DIR} ${imgui_SOURCE_DIR}/backends)
target_link_libraries(test_create_tool PRIVATE imgui tangent_epoxy)
target_compile_options(test_create_tool PRIVATE ${TANGENT_WARNINGS_LOOSE})
add_test(NAME create_tool COMMAND test_create_tool)

add_executable(test_guard tests/test_guard.cpp
  src/scene/scene.cpp src/scene/feature.cpp src/mesh/boolean.cpp src/mesh/health.cpp
  ${TANGENT_GEOM} src/mesh/operations.cpp src/mesh/halfedge.cpp src/mesh/primitives.cpp
  src/app/camera.cpp)
target_include_directories(test_guard PRIVATE src)
target_compile_options(test_guard PRIVATE ${TANGENT_WARNINGS})
add_test(NAME guard COMMAND test_guard)

add_executable(test_drag tests/test_drag.cpp
  src/app/drag_axis.cpp src/app/camera.cpp
  src/render/renderer.cpp src/render/shader.cpp src/render/gpu_mesh.cpp
  src/scene/scene.cpp src/scene/feature.cpp src/mesh/boolean.cpp src/mesh/health.cpp
  ${TANGENT_GEOM} src/mesh/operations.cpp src/mesh/halfedge.cpp src/mesh/primitives.cpp)
target_include_directories(test_drag PRIVATE src)
target_link_libraries(test_drag PRIVATE tangent_epoxy)
target_compile_options(test_drag PRIVATE ${TANGENT_WARNINGS})
add_test(NAME drag COMMAND test_drag)

add_executable(test_snap tests/test_snap.cpp
  src/app/snap.cpp src/app/camera.cpp
  src/scene/scene.cpp src/scene/feature.cpp src/mesh/boolean.cpp src/mesh/health.cpp
  ${TANGENT_GEOM} src/mesh/operations.cpp src/mesh/halfedge.cpp src/mesh/primitives.cpp)
target_include_directories(test_snap PRIVATE src)
target_link_libraries(test_snap PRIVATE tangent_epoxy)
target_compile_options(test_snap PRIVATE ${TANGENT_WARNINGS})
add_test(NAME snap COMMAND test_snap)

add_executable(test_lod tests/test_lod.cpp
  src/render/lod.cpp src/app/camera.cpp
  src/scene/scene.cpp src/scene/feature.cpp src/mesh/boolean.cpp src/mesh/health.cpp
  ${TANGENT_GEOM} src/mesh/operations.cpp src/mesh/halfedge.cpp src/mesh/primitives.cpp)
target_include_directories(test_lod PRIVATE src)
target_link_libraries(test_lod PRIVATE tangent_epoxy)
target_compile_options(test_lod PRIVATE ${TANGENT_WARNINGS})
add_test(NAME lod COMMAND test_lod)

# The exact backend's own suite. Only built when there is a backend to test:
# the seam contract in test_body covers what both backends share, and this
# covers what only this one can do.
if(TANGENT_BREP)
  add_executable(test_brep tests/test_brep.cpp
    src/mesh/health.cpp src/mesh/boolean.cpp ${TANGENT_GEOM}
    src/mesh/operations.cpp src/mesh/halfedge.cpp src/mesh/primitives.cpp)
  target_include_directories(test_brep PRIVATE src)
  target_compile_options(test_brep PRIVATE ${TANGENT_WARNINGS})
  add_test(NAME brep COMMAND test_brep)
endif()

# Not a test: it bakes the toolbar icons out of the kernel. Run it by hand when
# the icons change, and commit what it produces.
add_executable(bake_icons EXCLUDE_FROM_ALL tools/bake_icons.cpp tools/icon_raster.cpp
  src/mesh/health.cpp src/mesh/boolean.cpp ${TANGENT_GEOM}
  src/mesh/operations.cpp src/mesh/halfedge.cpp src/mesh/primitives.cpp)
target_include_directories(bake_icons PRIVATE src tools)
target_compile_options(bake_icons PRIVATE ${TANGENT_WARNINGS})

# Not a test: a stopwatch for the heavy paths. Run it by hand.
add_executable(bench EXCLUDE_FROM_ALL tests/bench.cpp
  src/scene/scene.cpp src/scene/feature.cpp src/mesh/health.cpp
  src/mesh/boolean.cpp ${TANGENT_GEOM} src/mesh/operations.cpp src/mesh/halfedge.cpp src/mesh/primitives.cpp
  src/render/lod.cpp src/app/camera.cpp)
target_include_directories(bench PRIVATE src)
target_link_libraries(bench PRIVATE tangent_epoxy)
