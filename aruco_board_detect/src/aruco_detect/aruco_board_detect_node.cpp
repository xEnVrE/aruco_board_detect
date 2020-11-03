#include <stdio.h>
#include <opencv2/aruco.hpp>
#include <vector>
#include <ros/ros.h>

#include "aruco_detect/aruco_board_detect_node.hpp"

void aruco_board_detect::imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    try
    {
        cv::imshow("view", cv_bridge::toCvShare(msg, "bgr8")->image);
        cv::waitKey(30);
    }
    catch(cv_bridge::Exception& e)
    {
        ROS_ERROR("Could not convert from '%s' to 'bgr8.", msg->encoding.c_str());
    }

}


int main(int argc, char** argv )
{
    ros::init(argc, argv, "image_listener_tutorial");
    ros::NodeHandle nh;

    cv::namedWindow("view");
    cv::startWindowThread();

    image_transport::ImageTransport it(nh);
    image_transport::Subscriber sub = it.subscribe("camera/color/image_raw", 1, aruco_board_detect::imageCallback);

    ros::spin();
    cv::destroyWindow("view");

}