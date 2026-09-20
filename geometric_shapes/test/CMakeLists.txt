file(TO_NATIVE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/resources" TEST_RESOURCES_DIR)
if(WIN32)
    # Correct directory separator for Windows
    string(REPLACE "\\" "\\\\" TEST_RESOURCES_DIR "${TEST_RESOURCES_DIR}")
endif()
configure_file(resources/config.h.in "${CMAKE_CURRENT_BINARY_DIR}/resources/config.h")
include_directories(${CMAKE_CURRENT_BINARY_DIR})

find_package(PkgConfig REQUIRED)
pkg_check_modules(GTEST REQUIRED gtest)

include_directories(${GTEST_INCLUDE_DIRS})
link_directories(${GTEST_LIBRARY_DIRS})

add_executable(test_basics test_basics.cpp)
target_link_libraries(test_basics ${PROJECT_NAME} ${Boost_LIBRARIES} ${GTEST_LIBRARIES} pthread)

add_executable(test_point_inclusion test_point_inclusion.cpp)
target_link_libraries(test_point_inclusion ${PROJECT_NAME} ${Boost_LIBRARIES} ${GTEST_LIBRARIES} pthread)

add_executable(test_bounding_sphere test_bounding_sphere.cpp)
target_link_libraries(test_bounding_sphere ${PROJECT_NAME} ${Boost_LIBRARIES} ${GTEST_LIBRARIES} pthread)

add_executable(test_bounding_sphere_mine test_bounding_sphere_mine.cpp)
target_link_libraries(test_bounding_sphere_mine ${PROJECT_NAME} ${Boost_LIBRARIES} ${GTEST_LIBRARIES} pthread)

add_executable(test_bounding_box test_bounding_box.cpp)
target_link_libraries(test_bounding_box ${PROJECT_NAME} ${Boost_LIBRARIES} ${LIBFCL_LIBRARIES} ${GTEST_LIBRARIES} pthread)

add_executable(test_bounding_box_mine test_bounding_box_mine.cpp)
target_link_libraries(test_bounding_box_mine ${PROJECT_NAME} ${Boost_LIBRARIES} ${LIBFCL_LIBRARIES} ${GTEST_LIBRARIES} pthread)

add_executable(test_bounding_cylinder test_bounding_cylinder.cpp)
target_link_libraries(test_bounding_cylinder ${PROJECT_NAME} ${Boost_LIBRARIES} ${GTEST_LIBRARIES} pthread)

add_executable(test_create_mesh test_create_mesh.cpp)
target_link_libraries(test_create_mesh ${PROJECT_NAME} ${Boost_LIBRARIES} ${GTEST_LIBRARIES} pthread)

add_executable(test_loaded_meshes test_loaded_meshes.cpp)
target_link_libraries(test_loaded_meshes ${PROJECT_NAME} ${Boost_LIBRARIES} ${GTEST_LIBRARIES} pthread)

add_executable(test_loaded_meshes_mine test_loaded_meshes_mine.cpp)
target_link_libraries(test_loaded_meshes_mine ${PROJECT_NAME} ${Boost_LIBRARIES} ${GTEST_LIBRARIES} pthread)

add_executable(test_shapes test_shapes.cpp)
target_link_libraries(test_shapes ${PROJECT_NAME} ${Boost_LIBRARIES} ${GTEST_LIBRARIES} pthread)

add_executable(test_ray_intersection test_ray_intersection.cpp)
target_link_libraries(test_ray_intersection ${PROJECT_NAME} ${Boost_LIBRARIES} ${GTEST_LIBRARIES} pthread)

add_executable(test_body_operations test_body_operations.cpp)
target_link_libraries(test_body_operations ${PROJECT_NAME} ${Boost_LIBRARIES} ${GTEST_LIBRARIES} pthread)

add_executable(test_meshes_obb test_meshes_obb.cpp)
target_link_libraries(test_meshes_obb ${PROJECT_NAME} ${Boost_LIBRARIES} ${GTEST_LIBRARIES} pthread)

add_test(NAME test_basics COMMAND test_basics)
add_test(NAME test_point_inclusion COMMAND test_point_inclusion)
add_test(NAME test_bounding_sphere COMMAND test_bounding_sphere)
add_test(NAME test_bounding_sphere_mine COMMAND test_bounding_sphere_mine)
add_test(NAME test_bounding_box COMMAND test_bounding_box)
add_test(NAME test_bounding_box_mine COMMAND test_bounding_box_mine)
add_test(NAME test_bounding_cylinder COMMAND test_bounding_cylinder)
add_test(NAME test_create_mesh COMMAND test_create_mesh)
add_test(NAME test_loaded_meshes COMMAND test_loaded_meshes)
add_test(NAME test_loaded_meshes_mine COMMAND test_loaded_meshes_mine)
add_test(NAME test_shapes COMMAND test_shapes)
add_test(NAME test_ray_intersection COMMAND test_ray_intersection)
add_test(NAME test_body_operations COMMAND test_body_operations)
add_test(NAME test_meshes_obb COMMAND test_meshes_obb)