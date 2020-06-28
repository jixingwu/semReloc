//
// Created by jixingwu on 2020/6/18.
//

#include "DataManager.h"

DataManager::DataManager(ros::NodeHandle &nh)
{
    this->nh = nh;
}

#define __DATAMANAGER_CALLBACK_PRINT__(msg)

void DataManager::camera_pose_callback(const nav_msgs::Odometry &msg) {
    if(pose_0_available == false){
        pose_0 = msg.header.stamp;
        pose_0_available = true;
    }
    m_buf.lock();
    __DATAMANAGER_CALLBACK_PRINT__(
            cout << TermColor::iBLUE() << "[DataManager/camera_pose_callback]"
                 << msg->header.stamp << "\t" << msg->header.stamp - pose_0 << TermColor::RESET() << endl;)
    pose_buf.push(msg);
    m_buf.unlock();
    return;
}
void DataManager::keyframe_bboxes_callback(const darknet_ros_msgs::BoundingBoxes &msg) {
    m_buf.lock();
    __DATAMANAGER_CALLBACK_PRINT__(
            cout << TermColor::iBLUE() << "[DataManager/keyframe_bboxes_callback]"
                << msg->header.stamp << "\t" << msg->header.stamp - pose_0 << TermColor::RESET() << endl;)
    keyframe_bboxes_buf.push(msg);
    m_buf.unlock();
    return;
}

void DataManager::frame_bboxes_callback(const darknet_ros_msgs::BoundingBoxes &msg) {
    m_buf.lock();
    __DATAMANAGER_CALLBACK_PRINT__(
            cout << TermColor::iBLUE() << "[DataManager/frame_bboxes_callback]"
                 << msg->header.stamp << "\t" << msg->header.stamp - pose_0 << TermColor::RESET() << endl;)
    frame_bboxes_buf.push(msg);
    m_buf.unlock();
    return;
}

void DataManager::leftImage_callback(const sensor_msgs::ImageConstPtr &msg) {
    m_buf.lock();
    __DATAMANAGER_CALLBACK_PRINT__(
            cout << TermColor::iBLUE() << "[DataManager/leftImage_callback]"
                 << msg->header.stamp << "\t" << msg->header.stamp - pose_0 << TermColor::RESET() << endl;)
    img_buf.push(msg);
    m_buf.unlock();
    return;
}

void DataManager::keyframe_image_callback(const sensor_msgs::ImageConstPtr &msg) {
    m_buf.lock();
    __DATAMANAGER_CALLBACK_PRINT__(
            cout << TermColor::iBLUE() << "[DataManager/keyframe_image_callback]"
                 << msg->header.stamp << "\t" << msg->header.stamp - pose_0 << TermColor::RESET() << endl;)
    keyimg_buf.push(msg);
    m_buf.unlock();
    return;
}