#pragma once

// ROS
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>

// OpenCV + ArUCO
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <mutex>


class ImageConverter
{
    // this class is supposed to catch all camera images and store them locally

    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    image_transport::Publisher image_pub_;

    cv_bridge::CvImagePtr current_img_ptr_;

    std::mutex image_mutex_;

    std::string debug_window_name_;

public:

    ImageConverter(ros::NodeHandle& nh);

    ~ImageConverter();

    void getCurrentImage(cv::Mat& cv_image);

    void imageAcquisitionCallback(const sensor_msgs::ImageConstPtr& msg);

};


class ArucoBoardDetector
{
    // this class is fed images and computes the pose of the board once in a while

};

class ArucoDetectNode
{
    // this class has an image converter and a board detector inside, sets up callbacks and calls the board detector once in a while
    // also publishes a tf pose with the board pose


};


namespace aruco_board_detect
{

    void imageCallback(const sensor_msgs::ImageConstPtr& msg);

}