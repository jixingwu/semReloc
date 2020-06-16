
/*******************************************************
 * please first launch KITTIOdomTest.cpp in VINS_Fusion of vins_estimator
 *
 *******************************************************/

#include <iostream>
#include <stdio.h>
#include <opencv2/opencv.hpp>
#include <cmath>
#include <string>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
//#include "estimator/estimator.h"
//#include "utility/visualization.h"

//#include "semMapping/SubscribeAndPublish.h"
//#include "semMapping/landmark.h"
#include "../include/landmark.h"
#include "../include/tracking.h"
#include "../include/frame.h"
//#include "semMapping/tracking.h"

//#include "detect_3d_cuboid/matrix_utils.h"
//#include "detect_3d_cuboid/detect_3d_cuboid.h"
//
//#include "line_lbd/line_lbd_allclass.h"

//#include "semMapping/frame/frame.h"

#define DEBUG

using namespace std;
using namespace Eigen;

//Estimator estimator;
Landmark landmark;
Tracking tracking;

int main(int argc, char** argv)
{

    //TODO: 写成单独node形式， 最好不在vins-fusion代码中写，独立出来

    ros::init(argc, argv, "vins_cuboid");
    ROS_INFO("cube_vo");
    ros::NodeHandle n("~");
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);

    ros::Publisher pubLeftImage = n.advertise<sensor_msgs::Image>("/leftImage", 1000);
    ros::Publisher pubRightImage = n.advertise<sensor_msgs::Image>("/rightImage", 1000);

    if(argc !=3){
        printf("ERROR: 参数不足 ");
        cout<<"argv[1]: "<<argv[1]<<endl;
        cout<<"argv[2]: "<<argv[2]<<endl;
        ROS_BREAK();
        return 1;
    }



//    SubscribeAndPublish subandpub;
//    detect_3d_cuboid detect_cuboid_obj;
//    line_lbd_detect line_lbd_obj;

    string config_file = argv[1];
    printf("dataPath:%s\n", argv[1]);
    string sequence = argv[2];
    printf("read sequence: %s\n", argv[2]);
    string dataPath = sequence + "/";

//    readParameters(config_file);
//    estimator.setParameter();
//    registerPub(n);

    tracking.framer.setCubParameter();//set detected cuboid and line based parameters

    //detect cuboid into mapDrawer and Map



    //load image times
    FILE* file;
    file = std::fopen((dataPath + "times.txt").c_str(), "r");
    if(file == NULL)
    {
        printf("cannot find file: %stimes.txt\n", dataPath.c_str());
        ROS_BREAK();
        return 0;
    }

    double imageTime;
    vector<double> imageTimeList;
    while(fscanf(file, "%lf", &imageTime) != EOF)
        imageTimeList.push_back(imageTime);
    std::fclose(file);

    string leftImagePath, rightImagePath;
    cv::Mat imLeft, imRight;

    for (size_t i = 0; i < imageTimeList.size(); ++i)//imageTimeLIst.size() =1000
    {
        if(ros::ok())
        {
            printf("\nprocess image %d\n", (int)i);
            stringstream ss;
            ss << setfill('0') << setw(6) << i;
            leftImagePath = dataPath + "image_0/" + ss.str() + ".png";
            rightImagePath = dataPath + "image_1/" + ss.str() + ".png";

            imLeft = cv::imread(leftImagePath, CV_LOAD_IMAGE_GRAYSCALE);
            sensor_msgs::ImagePtr imLeftMsg = cv_bridge::CvImage(std_msgs::Header(), "mono8", imLeft).toImageMsg();
            imLeftMsg->header.stamp = ros::Time(imageTimeList[i]);
            pubLeftImage.publish(imLeftMsg);

            imRight = cv::imread(rightImagePath, CV_LOAD_IMAGE_GRAYSCALE);
            sensor_msgs::ImagePtr imRightMsg = cv_bridge::CvImage(std_msgs::Header(), "mono8", imRight).toImageMsg();
            imRightMsg->header.stamp = ros::Time(imageTimeList[i]);
            pubRightImage.publish(imRightMsg);

            estimator.inputImage(imageTimeList[i], imLeft, imRight);
            Eigen::Matrix4d pose;
            estimator.getPoseInWorldFrame(pose);

            Eigen::Matrix4d cam_transToGround;
            estimator.getPoseInWorldFrame(cam_transToGround);//get frame pose in world

            tracking.inputImage(imLeft);
            tracking.inputCamPose(cam_transToGround);



            
            //TODO 每一帧检测cub 并且pub出去，再pub bbox然后就处理吧
            // 最后和pose发送给visualization_msgs
        }
    }
#ifdef DEBUG
    printf("图片处理完毕\n");
#endif

    return 0;
}
