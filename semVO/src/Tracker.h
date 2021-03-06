//
// Created by jixingwu on 2020/3/16.
//

#ifndef SRC_TRACKER_H
#define SRC_TRACKER_H

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

#include "detect_3d_cuboid/detect_3d_cuboid.h"
#include "detect_3d_cuboid/object_3d_util.h"
#include "line_lbd/line_lbd_allclass.h"
#include "Object_landmark.h"

#include "GraphMatching.h"
#include "DataManager.h"
#include "Frame.h"
#include "Converter.h"

#include "g2o_Object.h"
#include "Object_landmark.h"

typedef Eigen::Matrix<double, 3, 8> Matrix38d;
typedef Eigen::Matrix<double, 4, 2> Matrix42d;
//typedef Eigen::Matrix<double, 9, 1> Vector9d;


class Tracking
{
private:

    ros::NodeHandle n;

    GraphMatching<TopoMetric_c, Node_c> graphmatching_h;
    DataManager dataManager;
    Frame framer;//用于对极约束和三角化

public:

    Tracking();
    ~Tracking();
    void inputImage(const cv::Mat& img);
    void inputImageMsg();// INPUT: img_buf
    void inputCamPose(Eigen::Matrix4d cam_pose);
    void inputFrameBboxes2trian(darknet_ros_msgs::BoundingBoxes frame_bboxes,
                                darknet_ros_msgs::BoundingBoxes frame_bboxes_next);
    void bboxes2CenterPpoints2f(darknet_ros_msgs::BoundingBoxes frame_bboxes, vector<cv::Point2f>& points);

    double computeError(Matrix42d keyframeCoor, Matrix42d frameCoor);

private:
    queue<nav_msgs::Odometry> pose_buf = dataManager.getCameraPose();
    queue<darknet_ros_msgs::BoundingBoxes> keyframe_bboxes_buf = dataManager.getKeyframeBboxes();
    queue<darknet_ros_msgs::BoundingBoxes> frame_bboxes_buf = dataManager.getFrameBboxes();
    queue<sensor_msgs::ImageConstPtr> img_buf = dataManager.getFrameImage();
    queue<sensor_msgs::ImageConstPtr> keyimg_buf = dataManager.getKeyframeImage();
    std::mutex m_buf;


public:
    cv::Mat InitToGround;//    Eigen::Matrix4d cam_transToGround;
    Eigen::Matrix3d Kalib;
    detect_3d_cuboid *detect_cuboid_obj;
    double obj_det_2d_thre;
    darknet_ros_msgs::BoundingBoxes frame_bboxes;
    darknet_ros_msgs::BoundingBoxes keyframe_bboxes;
    ros::Time img_t;

    line_lbd_detect line_lbd_obj;
    std::vector<ObjectSet> frames_cuboid;

    bool whether_save_online_detected_cuboids;
    bool whether_save_final_optimized_cuboids;
    bool has_detected_cuboid;

public:
    void Track();// main tracking function. input sensor dataset;
    void CreateNewKeyFrame(cv::Mat img, uint32_t imgID);
    void DetectCuboid(const cv::Mat& raw_image, cv::Mat camera_pose);
    void AssociateCuboid();
    bool MatchCuboid(darknet_ros_msgs::BoundingBoxes keyframe_bboxes, darknet_ros_msgs::BoundingBoxes frame_bboxes);//keyframe_bboxes, frame_bboxes;
    cv::Mat setImageFromMsg(const sensor_msgs::ImageConstPtr msg);
    visualization_msgs::MarkerArray cuboids_to_marker(cuboid* raw_cuboid, Vector3d rgbcolor);
    void cuboid_corner_to_marker(const Matrix38d& cube_corners, visualization_msgs::Marker& marker, int bodyOrfront);




//    cv::Mat TrackMonocular(const cv::Mat &im, const double &timestamp, int msg_seq_id=-1);// System::

};

#endif //SRC_TRACKER_H
