# Tangent - Stage 0 spike. Throwaway: nothing here is meant to ship.
#
# Standalone on purpose. The main build must not gain an OCCT dependency
# before the gate decision is made, so this project is configured by hand:
#
#   cmake -S spike -B spike/build -DCMAKE_BUILD_TYPE=Release \
#         -DCMAKE_PREFIX_PATH=<prefix holding lib/cmake/opencascade>
#   cmake --build spike/build -j
#
cmake_minimum_required(VERSION 3.16)
project(tangent_spike CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(OpenCASCADE REQUIRED CONFIG)
message(STATUS "OpenCASCADE ${OpenCASCADE_VERSION} from ${OpenCASCADE_INSTALL_PREFIX}")

# Only the modelling stack. Deliberately no visualization library: the whole
# dependency question for this project is whether the geometry libraries can be
# taken without dragging in vtk, tcl/tk, freetype and X.
set(OCCT_LIBS
  TKernel TKMath TKG2d TKG3d TKGeomBase TKGeomAlgo TKBRep TKTopAlgo
  TKPrim TKBO TKBool TKFillet TKOffset TKShHealing TKMesh)

add_executable(occt_probe occt_probe.cpp)
target_link_libraries(occt_probe PRIVATE ${OCCT_LIBS})
target_compile_options(occt_probe PRIVATE -Wall -Wextra)

# The comparison harness. It links the shipping mesh kernel straight out of
# ../src so that the control is the code that actually runs today, not a copy
# of it that could drift.
set(TG ${CMAKE_CURRENT_SOURCE_DIR}/../src)
add_executable(compare
  main.cpp part_spec.cpp backend_occt.cpp backend_mesh.cpp
  ${TG}/mesh/halfedge.cpp ${TG}/mesh/primitives.cpp ${TG}/mesh/operations.cpp
  ${TG}/mesh/boolean.cpp ${TG}/mesh/health.cpp)
target_include_directories(compare PRIVATE ${TG} ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(compare PRIVATE ${OCCT_LIBS})
target_compile_options(compare PRIVATE -Wall -Wextra)

# The naming experiment: the one piece of the spike whose result decides
# whether a parametric history can be carried on a B-rep at all.
add_executable(naming naming_main.cpp naming.cpp)
target_include_directories(naming PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(naming PRIVATE ${OCCT_LIBS})
target_compile_options(naming PRIVATE -Wall -Wextra)

add_executable(tess tess.cpp)
target_link_libraries(tess PRIVATE ${OCCT_LIBS})
target_compile_options(tess PRIVATE -Wall -Wextra)

add_executable(tess2 tess2.cpp)
target_link_libraries(tess2 PRIVATE ${OCCT_LIBS})
target_compile_options(tess2 PRIVATE -Wall -Wextra)

add_executable(bool_perf bool_perf.cpp)
target_link_libraries(bool_perf PRIVATE ${OCCT_LIBS})
target_compile_options(bool_perf PRIVATE -Wall -Wextra)
