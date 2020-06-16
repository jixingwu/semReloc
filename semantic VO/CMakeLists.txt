cmake_minimum_required(VERSION 2.8.3)
project(vins)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11")
#-DEIGEN_USE_MKL_ALL")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    geometry_msgs
    nav_msgs
    tf
    cv_bridge
    camera_models
    image_transport
        darknet_ros_msgs
        )

find_package(OpenCV REQUIRED)

# message(WARNING "OpenCV_VERSION: ${OpenCV_VERSION}")

find_package(Ceres REQUIRED)

include_directories(${catkin_INCLUDE_DIRS} ${CERES_INCLUDE_DIRS})

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
find_package(Eigen3)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

#find_package(G2O REQUIRED)
#include_directories(${G2O_INCLUDE_DIRS})

catkin_package(
        INCLUDE_DIRS
        CATKIN_DEPENDS
        LIBRARIES
        std_msgs
        geometry_msgs
        nav_msgs
        camera_models
        image_transport
        darknet_ros_msgs
        tf
        cv_bridge

)

add_library(vins_lib
    src/estimator/parameters.cpp
    src/estimator/estimator.cpp
    src/estimator/feature_manager.cpp
    src/factor/pose_local_parameterization.cpp
    src/factor/projectionTwoFrameOneCamFactor.cpp
    src/factor/projectionTwoFrameTwoCamFactor.cpp
    src/factor/projectionOneFrameTwoCamFactor.cpp
    src/factor/marginalization_factor.cpp
    src/utility/utility.cpp
    src/utility/visualization.cpp
    src/utility/CameraPoseVisualization.cpp
    src/initial/solve_5pts.cpp
    src/initial/initial_aligment.cpp
    src/initial/initial_sfm.cpp
    src/initial/initial_ex_rotation.cpp
    src/featureTracker/feature_tracker.cpp)

add_library(vins_lib_cub
        src/detect_3d_cuboid/box_proposal_detail.cpp
        src/detect_3d_cuboid/matrix_utils.cpp
        src/detect_3d_cuboid/object_3d_util.cpp

        src/line_lbd/binary_descriptor.cpp
        src/line_lbd/binary_descriptor_matcher.cpp
        src/line_lbd/draw.cpp
        src/line_lbd/line_lbd_allclass.cpp
        src/line_lbd/lsd.cpp
        src/line_lbd/LSDDetector.cpp

        src/semMapping/frame/frame.cpp
        src/semMapping/matchBBoxes/TopoMetric.cpp
        src/semMapping/landmark.cpp
        src/semMapping/map_drawer.cpp
        src/semMapping/map.cpp
        src/semMapping/tracking.cpp
        )

target_link_libraries(vins_lib ${catkin_LIBRARIES} ${OpenCV_LIBS} ${CERES_LIBRARIES} ${G2O_LIBRARIES})


add_executable(vins_node src/rosNodeTest.cpp)
target_link_libraries(vins_node vins_lib) 

add_executable(kitti_odom_test src/KITTIOdomTest.cpp)
target_link_libraries(kitti_odom_test vins_lib) 

add_executable(kitti_gps_test src/KITTIGPSTest.cpp)
target_link_libraries(kitti_gps_test vins_lib)

add_executable(kitti_cub_test src/KITTICubTest.cpp src/semMapping/frame/frame.cpp src/semMapping/frame/frame.h src/relocalization/NodeDataManager.cpp src/relocalization/NodeDataManager.h src/relocalization/PoseGraphSLAM.cpp src/relocalization/PoseGraphSLAM.h)
target_link_libraries(kitti_cub_test vins_lib vins_lib_cub)

