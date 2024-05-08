###########################################################
#                                                         #
# Copyright (c)                                           #
#                                                         #
# The Verifiable & Control-Theoretic Robotics (VECTR) Lab #
# University of California, Los Angeles                   #
#                                                         #
# Authors: Kenny J. Chen, Ryan Nemiroff, Brett T. Lopez   #
# Contact: {kennyjchen, ryguyn, btlopez}@ucla.edu         #
#  Mpdified by piluohong SCAU                                                       #
###########################################################

cmake_minimum_required(VERSION 3.12.4)
project(hc_lio)

# add_compile_options(-std=c++14)
set(CMAKE_CXX_FLAGS "-std=c++14")
set(CMAKE_BUILD_TYPE "Release")
# set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g -pthread")

add_definitions(-w)
add_definitions(-DROOT_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/\")
set(PCL_DIR "/home/h/ku/pcl-1.12.0/share/pcl-1.12") #18.04

find_package(PCL 1.12.0 REQUIRED QUIET) 
find_package(GTSAM REQUIRED QUIET)
find_package(Eigen3 REQUIRED QUIET)
find_package(octomap REQUIRED QUIET)
add_definitions(-DOCTOMAP_NODEBUGOUT)
find_package(PythonLibs 3.6)
# find_package (GeographicLib REQUIRED)

include(FindOpenMP)
if(OPENMP_FOUND)
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
  set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
else(OPENMP_FOUND)
  message("ERROR: OpenMP could not be found.")
endif(OPENMP_FOUND)

set(CMAKE_THREAD_PREFER_PTHREAD TRUE)
set(THREADS_PREFER_PTHREAD_FLAG TRUE)
find_package(Threads REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  pcl_ros
  octomap_ros
  octomap_msgs
  livox_ros_driver2
  visualization_msgs
  tf2_ros
)
# include_directories (${GeographicLib_INCLUDE_DIRS})

add_service_files(
  DIRECTORY srv
  FILES
  save_pcd.srv
)

generate_messages(
  DEPENDENCIES
  sensor_msgs
  std_msgs
  geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS
    roscpp
    std_msgs
    sensor_msgs
    geometry_msgs
    pcl_ros
    nav_msgs
    visualization_msgs
    livox_ros_driver2
    tf2_ros
  INCLUDE_DIRS
    include
  DEPENDS PCL GTSAM OCTOMAP 
  LIBRARIES
    ${PROJECT_NAME}
    nano_gicp
    nanoflann
    fast_gicp
)

include_directories(
                    include
                    ${catkin_INCLUDE_DIRS}
                    ${PCL_INCLUDE_DIRS}
                    ${EIGEN3_INCLUDE_DIRS}
                    ${GTSAM_INCLUDE_DIRS}
                    #${GEOGRAPHICLIB_INCLUDE_DIRS}
                    ${PYTHON_INCLUDE_DIRS})
link_directories(
                  include
                  ${catkin_LIBRARIES}
                  ${PCL_LIBRARY_DIRS}
                  ${GTSAM_LIBRARY_DIRS}
                  #${GEOGRAPHICLIB_LIBRARY_DIRS}
                  ${PYTHON_LIBRARY_DIRS})


                    
# include_directories(SYSTEM ${catkin_INCLUDE_DIRS})

# Not all machines have <cpuid.h> available
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)
file(WRITE ${CMAKE_BINARY_DIR}/test_cpuid.cpp "#include <cpuid.h>")
try_compile(HAS_CPUID ${CMAKE_BINARY_DIR} ${CMAKE_BINARY_DIR}/test_cpuid.cpp)
file(REMOVE ${CMAKE_BINARY_DIR}/test_cpuid.cpp)
if(HAS_CPUID)
  add_compile_definitions(HAS_CPUID)
endif()

# NanoFLANN
add_library(nanoflann STATIC
  src/nano_gicp/nanoflann.cc
)
target_link_libraries(nanoflann ${PCL_LIBRARIES})

add_library(nano_gicp STATIC
  src/nano_gicp/lsq_registration.cc
  src/nano_gicp/nano_gicp.cc
)
target_link_libraries(nano_gicp ${PCL_LIBRARIES} ${OpenMP_LIBS} nanoflann)

add_library(fast_gicp STATIC
src/fvgicp/fast_gicp.cpp  
src/fvgicp/dwvgicp.cpp  
src/fvgicp/lsq_registration.cpp
include/fvgicp/dwvgicp.hpp
include/fvgicp/fast_gicp.hpp
include/fvgicp/lsq_registration.hpp
include/fvgicp/dwvgicp_impl.hpp
)
target_link_libraries(fast_gicp ${PCL_LIBRARIES} ${OpenMP_LIBS})

# Odometry Node
add_executable(dlio_odom_node src/odom/odom_node.cc include/ikd-Tree/ikd_Tree.cpp src/odom/odom.cc) 

add_dependencies(dlio_odom_node ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)
target_compile_options(dlio_odom_node PRIVATE ${OpenMP_FLAGS})
target_link_libraries(dlio_odom_node  ${catkin_LIBRARIES} ${GTSAM_LIBRARIES} ${PCL_LIBRARIES} ${OpenMP_CXX_FLAGS} ${GeographicLib_LIBRARIES} ${OCTOMAP_LIBRARIES}
${PYTHON_LIBRARIES} Threads::Threads nano_gicp fast_gicp gtsam)

#message(${OpenMP_LIBS})

# # Mapping Node
add_executable (dlio_map_node src/odom/map_node.cc src/odom/map.cc)
add_dependencies(dlio_map_node ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)
target_compile_options(dlio_map_node PRIVATE ${OpenMP_FLAGS})
target_link_libraries(dlio_map_node ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenMP_CXX_FLAGS} Threads::Threads)

# Binaries
install( TARGETS dlio_odom_node dlio_map_node
         DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION} ) #
install( DIRECTORY cfg launch
         DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION} )
