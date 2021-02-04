cmake_minimum_required(VERSION 3.14)

#get options and compiling env
include(cmake/options.cmake)
include(cmake/linker.cmake)
include(cmake/ccache.cmake)

project(GraphPatchCalculator)
project(GraphPatchCalculator LANGUAGES CXX)

#set C++ standard to C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

############################
###get external projects
############################
include(cmake/fmt.cmake)
include(cmake/tbb.cmake)
include(cmake/span.cmake)
include(cmake/CLI11.cmake)
include(cmake/progress.cmake)
include(cmake/nlohmann.cmake)

message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")

if(CMAKE_BUILD_TYPE STREQUAL "Release")
  set(CMAKE_INTERPROCEDURAL_OPTIMIZATION TRUE)
endif()

# needed for multithreading
find_package(Threads REQUIRED)

include(cmake/flags.cmake)

#############################################
## library with all the logic which
## can later be easily linked against test
#############################################
add_library(GraphPatchCalculatorSrc "")

target_sources(GraphPatchCalculatorSrc
  PUBLIC
  ${CMAKE_CURRENT_LIST_DIR}/include/graph/Graph.hpp

  ${CMAKE_CURRENT_LIST_DIR}/include/utils/Utils.hpp
  ${CMAKE_CURRENT_LIST_DIR}/include/utils/Timer.hpp
  ${CMAKE_CURRENT_LIST_DIR}/include/utils/ProgramOptions.hpp

  ${CMAKE_CURRENT_LIST_DIR}/include/selection/NodeSelection.hpp
  ${CMAKE_CURRENT_LIST_DIR}/include/selection/SelectionCenterCalculator.hpp
  ${CMAKE_CURRENT_LIST_DIR}/include/selection/NodeSelectionCalculator.hpp
  ${CMAKE_CURRENT_LIST_DIR}/include/selection/FullNodeSelectionCalculator.hpp
  ${CMAKE_CURRENT_LIST_DIR}/include/selection/SelectionLookup.hpp
  ${CMAKE_CURRENT_LIST_DIR}/include/selection/SelectionOptimizer.hpp

  ${CMAKE_CURRENT_LIST_DIR}/include/pathfinding/Path.hpp
  ${CMAKE_CURRENT_LIST_DIR}/include/pathfinding/Dijkstra.hpp
  ${CMAKE_CURRENT_LIST_DIR}/include/pathfinding/CachingDijkstra.hpp
  ${CMAKE_CURRENT_LIST_DIR}/include/pathfinding/DijkstraQueue.hpp
  ${CMAKE_CURRENT_LIST_DIR}/include/pathfinding/Distance.hpp

  PRIVATE
  src/graph/Graph.cpp

  src/selection/NodeSelection.cpp
  src/selection/SelectionLookup.cpp
  src/selection/SelectionOptimizer.cpp

  src/utils/ProgramOptions.cpp

  src/pathfinding/Path.cpp
  src/pathfinding/Dijkstra.cpp
  src/pathfinding/CachingDijkstra.cpp
  )

# add the dependencies of the target to enforce
# the right order of compiling
add_dependencies(GraphPatchCalculatorSrc fmt-project)
add_dependencies(GraphPatchCalculatorSrc nlohmann-project)
add_dependencies(GraphPatchCalculatorSrc tbb-project)
add_dependencies(GraphPatchCalculatorSrc span-lite-project)
add_dependencies(GraphPatchCalculatorSrc CLI11-project)
add_dependencies(GraphPatchCalculatorSrc progress-cpp-project)

# make headers available
target_include_directories(GraphPatchCalculatorSrc PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  fmt
  tbb
  ${CLI11_INCLUDE_DIR}
  ${PROGRESS_CPP_INCLUDE_DIR}
  ${SPAN_LITE_INCLUDE_DIR}
  ${NLOHMANN_INCLUDE_DIR}
  )

#link against libarys
target_link_libraries(GraphPatchCalculatorSrc LINK_PUBLIC
  fmt
  tbb)

target_link_libraries(GraphPatchCalculatorSrc PRIVATE
  ${CMAKE_THREAD_LIBS_INIT})


###############################
## THE ACTUAL BINARY
###############################
add_executable(GraphPatchCalculator src/main.cpp)

# make headers available
target_include_directories(GraphPatchCalculator PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  fmt
  tbb
  ${CLI11_INCLUDE_DIR}
  ${PROGRESS_CPP_INCLUDE_DIR}
  ${SPAN_LITE_INCLUDE_DIR}
  ${NLOHMANN_INCLUDE_DIR}
  )

#link against libarys
target_link_libraries(GraphPatchCalculator LINK_PUBLIC
  GraphPatchCalculatorSrc
  fmt
  tbb
  ${CMAKE_THREAD_LIBS_INIT})

# add the dependencies of the target to enforce
# the right order of compiling
add_dependencies(GraphPatchCalculator GraphPatchCalculatorSrc)
