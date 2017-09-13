cmake_minimum_required(VERSION 2.8.3)
project(test_g2o)

set(CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake_modules")

#ds load Eigen library
find_package(Eigen3 REQUIRED)

#ds attempt to locate a g2o package through srrg_cmake_modules
find_package(G2O REQUIRED)

#ds load suite sparse for pose graph optimization
find_package(SuiteSparse REQUIRED)

#ds set includes
include_directories(${EIGEN3_INCLUDE_DIR} ${G2O_INCLUDE_DIR} ${CSPARSE_INCLUDE_DIR})

#ds set libraries
set(G2O_LIBRARIES ${G2O_CORE_LIBRARY} ${G2O_TYPES_SLAM3D} ${G2O_STUFF_LIBRARY})

#ds info
message("g2o headers: ${G2O_INCLUDE_DIR}")
message("g2o libraries: ${G2O_LIBRARIES}")

#ds set flags
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} --std=c++11 -Wall")

#ds g2o testing
add_executable(test_g2o_compatibility test_g2o_compatibility.cpp)
target_link_libraries(test_g2o_compatibility ${G2O_LIBRARIES} ${CSPARSE_LIBRARY})
