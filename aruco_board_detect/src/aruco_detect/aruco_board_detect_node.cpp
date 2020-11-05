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

ImageConverter::ImageConverter(ros::NodeHandle& nh) : it_(nh)
{
    // Subscribe to the camera image topic

    image_sub_ = it_.subscribe("/camera/color/image_raw", 1, &ImageConverter::imageAcquisitionCallback, this);

    debug_window_name_ = "Input camera image";
    cv::namedWindow(debug_window_name_);

}

ImageConverter::~ImageConverter()
{
    cv::destroyWindow(debug_window_name_);
}

void ImageConverter::getCurrentImage(cv::Mat& cv_image)
{
    // Assign the image field of the current cv_bridge::cvImage

    image_mutex_.lock();

    cv_image = current_img_ptr_->image;

    image_mutex_.unlock();

    return;
}

void ImageConverter::imageAcquisitionCallback(const sensor_msgs::ImageConstPtr& msg)
{
    // Copy the image from the sensor_msg and store it internally

    image_mutex_.lock();

    try
    {
        current_img_ptr_ = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

        cv::imshow(debug_window_name_, current_img_ptr_->image);
        cv::waitKey(3);
    }
    catch (cv_bridge::Exception& exc)
    {
        ROS_ERROR("Caught cv_bridge exception: %s", exc.what());
    }

    image_mutex_.unlock();

    return;

}

// int main(int argc, char** argv )
// {
//     ros::init(argc, argv, "image_listener_tutorial");
//     ros::NodeHandle nh;

//     cv::namedWindow("view");
//     cv::startWindowThread();

//     image_transport::ImageTransport it(nh);
//     image_transport::Subscriber sub = it.subscribe("camera/color/image_raw", 1, aruco_board_detect::imageCallback);

//     ros::spin();
//     cv::destroyWindow("view");

// }

int main(int argc, char** argv)
{
    ros::init(argc, argv, "aruco_board_detect_node");
    ros::NodeHandle nh;

    ImageConverter image_converter(nh);

    if (!ros::isShuttingDown)
        ros::spinOnce();

    return 0;
}
