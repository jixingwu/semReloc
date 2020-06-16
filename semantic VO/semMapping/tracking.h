//
// Created by jixingwu on 2020/3/16.
//

#ifndef SRC_TRACKING_H
#define SRC_TRACKING_H

#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#include <iostream>
#include <mutex>
#include <vector>
#include <Eigen/Core>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include "darknet_ros_msgs/BoundingBoxes.h"
#include "darknet_ros_msgs/BoundingBox.h"

#include "../detect_3d_cuboid/detect_3d_cuboid.h"
#include "../detect_3d_cuboid/object_3d_util.h"
#include "../line_lbd/line_lbd_allclass.h"
#include "../object_slam/Object_landmark.h"

#include "matchBBoxes/GraphMatching.h"

#include "frame/frame.h"

typedef Eigen::Matrix<double, 3, 8> Matrix38d;
typedef Eigen::Matrix<double, 4, 2> Matrix42d;


class Tracking
{
private:
    detect_3d_cuboid detect_cuboid_obj;
    line_lbd_detect line_lbd_obj;

public:
    Tracking() = default;
    ~Tracking() = default;
    void inputImage(const cv::Mat& img);
    void inputCamPose(Eigen::Matrix4d cam_pose);
    void inputFrameBboxes2trian(darknet_ros_msgs::BoundingBoxes frame_bboxes,
                                darknet_ros_msgs::BoundingBoxes frame_bboxes_next);
    void bboxes2CenterPpoints2f(darknet_ros_msgs::BoundingBoxes frame_bboxes, vector<cv::Point2f>& points);

    void DetectCuboid(const cv::Mat& raw_image);
//    void AssociateCuboid();
    bool MatchCuboid(darknet_ros_msgs::BoundingBoxes keyframe_bboxes, darknet_ros_msgs::BoundingBoxes frame_bboxes);//keyframe_bboxes, frame_bboxes;
    double computeError(Matrix42d keyframeCoor, Matrix42d frameCoor);

    Eigen::Matrix4d cam_transToGround;
    darknet_ros_msgs::BoundingBoxes frame_bboxes;
    darknet_ros_msgs::BoundingBoxes keyframe_bboxes;
    std::vector<ObjectSet> frames_cuboid;

    GraphMatching<TopoMetric_c, Node_c> graphmatching_h;

    frame framer;//用于对极约束和三角化


};

#endif //SRC_TRACKING_H
