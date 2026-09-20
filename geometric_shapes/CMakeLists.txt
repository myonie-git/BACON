cmake_minimum_required(VERSION 3.16)
project(geometric_shapes)

# Set compile options
set(PROJECT_COMPILE_OPTIONS
  -Wall
  -Wextra
  -Wwrite-strings
  -Wunreachable-code
  -Wpointer-arith
  -Wredundant-decls
  -Wno-unused-parameter
)

list(APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake/")

# Set default build type to Debug if not specified
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Debug CACHE STRING "Build type (Debug, Release, RelWithDebInfo, MinSizeRel)" FORCE)
endif()

# used for finding Assimp and FCL
find_package(PkgConfig REQUIRED)

# TODO: Remove policy fix when assimp 5.1 is available
set(CMAKE_POLICY_DEFAULT_CMP0012 NEW)
find_package(ASSIMP QUIET)
if(NOT ASSIMP_FOUND)
  pkg_check_modules(ASSIMP_PC REQUIRED assimp)
  set(ASSIMP_INCLUDE_DIRS ${ASSIMP_PC_INCLUDE_DIRS})
endif()

find_library(ASSIMP_ABS_LIBRARIES NAMES ${ASSIMP_LIBRARIES} assimp HINTS ${ASSIMP_LIBRARY_DIRS} ${ASSIMP_PC_LIBRARY_DIRS})
set(ASSIMP_LIBRARIES "${ASSIMP_ABS_LIBRARIES}")

find_package(Boost REQUIRED system filesystem)

find_package(console_bridge REQUIRED)

find_package(Eigen3 REQUIRED)

find_package(octomap REQUIRED)

pkg_check_modules(LIBFCL_PC REQUIRED fcl)
find_path(LIBFCL_INCLUDE_DIRS fcl/config.h HINTS ${LIBFCL_PC_INCLUDE_DIR} ${LIBFCL_PC_INCLUDE_DIRS})
set(LIBFCL_LIBRARIES)
foreach(_lib ${LIBFCL_PC_LIBRARIES})
  find_library(_lib_${_lib} ${_lib} HINTS ${LIBFCL_PC_LIBRARY_DIRS})
  list(APPEND LIBFCL_LIBRARIES ${_lib_${_lib}})
endforeach()

find_package(QHULL REQUIRED)
find_package(random_numbers REQUIRED)
find_package(resource_retriever REQUIRED)

include_directories(include)
include_directories(SYSTEM
  ${EIGEN3_INCLUDE_DIR} ${Boost_INCLUDE_DIR} ${ASSIMP_INCLUDE_DIRS} ${OCTOMAP_INCLUDE_DIRS}
  ${QHULL_INCLUDE_DIRS} ${LIBFCL_INCLUDE_DIRS} ${console_bridge_INCLUDE_DIRS}
  ${random_numbers_INCLUDE_DIRS} ${resource_retriever_INCLUDE_DIRS})

add_library(${PROJECT_NAME}
  src/aabb.cpp
  src/bodies.cpp
  src/body_operations.cpp
  src/mesh_operations.cpp
  src/obb.cpp
  src/shape_extents.cpp
  src/shape_operations.cpp
  src/shape_to_marker.cpp
  src/shapes.cpp
)
target_compile_options(${PROJECT_NAME} PRIVATE ${PROJECT_COMPILE_OPTIONS})
set_target_properties(${PROJECT_NAME} PROPERTIES VERSION 1.0 SOVERSION 1)

target_link_libraries(${PROJECT_NAME} ${ASSIMP_LIBRARIES} ${QHULL_LIBRARIES} ${console_bridge_LIBRARIES} ${Boost_LIBRARIES} ${LIBFCL_LIBRARIES} ${random_numbers_LIBRARIES} ${resource_retriever_LIBRARIES})


if(BACON_BUILD_GEOMETRIC_TESTS)
    add_compile_definitions(BACON_SOURCE_DIR="${CMAKE_SOURCE_DIR}")
    enable_testing()
    add_subdirectory(test)
endif()


install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
)
