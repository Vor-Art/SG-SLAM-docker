cmake_minimum_required(VERSION 2.8)
project(sg-slam)

# ----------------------------- Build type ------------------------------------
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")

set(CMAKE_C_FLAGS   "${CMAKE_C_FLAGS}  -Wall -O3")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -O3")

# -------------------------- C++11 check --------------------------------------
include(CheckCXXCompilerFlag)
check_cxx_compiler_flag("-std=c++11" COMPILER_SUPPORTS_CXX11)
if(NOT COMPILER_SUPPORTS_CXX11)
  message(FATAL_ERROR "Compiler ${CMAKE_CXX_COMPILER} has no C++11 support.")
endif()
set(CMAKE_CXX_STANDARD 11)
add_definitions(-DCOMPILEDWITHC11)

# -------------------------- Catkin & ROS -------------------------------------
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  cv_bridge
  image_geometry
  message_generation
  message_filters
  tf
)

# -------------------------- Third-party libs ---------------------------------
find_package(PkgConfig REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Pangolin REQUIRED)
find_package(PCL REQUIRED)
find_package(octomap REQUIRED)

# ncnn – path injected by Dockerfile (ENV ncnn_DIR)
if(NOT ncnn_DIR)
  set(ncnn_DIR "$ENV{ncnn_DIR}" CACHE PATH "Directory that contains ncnnConfig.cmake")
endif()
find_package(ncnn REQUIRED)

# -------------------------- Includes & dirs ----------------------------------
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}
  ${PROJECT_SOURCE_DIR}/include
  ${EIGEN3_INCLUDE_DIR}
  ${Pangolin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${OCTOMAP_INCLUDE_DIRS}
)
add_definitions(${PCL_DEFINITIONS})
link_directories(${PCL_LIBRARY_DIRS})

# -------------------------- Output folders -----------------------------------
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/run_examples)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)

# -------------------------- Core library -------------------------------------
add_library(${PROJECT_NAME} SHARED
  src/System.cc
  src/Tracking.cc
  src/LocalMapping.cc
  src/LoopClosing.cc
  src/ORBextractor.cc
  src/ORBmatcher.cc
  src/FrameDrawer.cc
  src/Converter.cc
  src/MapPoint.cc
  src/KeyFrame.cc
  src/Map.cc
  src/MapDrawer.cc
  src/Optimizer.cc
  src/PnPsolver.cc
  src/Frame.cc
  src/KeyFrameDatabase.cc
  src/Sim3Solver.cc
  src/Initializer.cc
  src/Viewer.cc
  src/Detector2D.cc
  src/Detector3D.cc
  src/ObjectDatabase.cc
  src/PointcloudMapping.cc
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  Eigen3::Eigen
  ${Pangolin_LIBRARIES}
  ${PROJECT_SOURCE_DIR}/Thirdparty/DBoW2/lib/libDBoW2.so
  ${PROJECT_SOURCE_DIR}/Thirdparty/g2o/lib/libg2o.so
  ${PCL_LIBRARIES}
  ncnn
  ${OCTOMAP_LIBRARIES}
  -lgomp
)

# -------------------------- Executables --------------------------------------
add_executable(sg_slam_tum        run_examples/rgbd_tum.cc)
add_executable(sg_slam_ros_rgbd   run_examples/ros_rgbd.cc)

target_link_libraries(sg_slam_tum       ${PROJECT_NAME})
target_link_libraries(sg_slam_ros_rgbd  ${PROJECT_NAME})

# --------------------------Catkin Package metadata ---------------------------------
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp sensor_msgs cv_bridge image_geometry tf
)

# -------------------------- Install rules ------------
install(TARGETS ${PROJECT_NAME} sg_slam_tum sg_slam_ros_rgbd
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY include/ DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

