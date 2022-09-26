#include <image_converter.h>

#include <sensor_msgs/image_encodings.h>

#include <opencv2/opencv.hpp>


ImageConverter::ImageConverter(ros::NodeHandle& nh) : it_(nh)
{
    // Subscribe to the camera image topic

    image_sub_ = it_.subscribe("input/image_raw", 1, &ImageConverter::imageAcquisitionCallback, this);

    // debug_window_name_ = "Input camera image";

    // if (show_debug_window_)
    //     cv::namedWindow(debug_window_name_);
}


ImageConverter::~ImageConverter()
{
    // if (show_debug_window_)
    //     cv::destroyWindow(debug_window_name_);
}


bool ImageConverter::getCurrentImage(cv::Mat& cv_image)
{

    image_mutex_.lock();

    if (current_img_ptr_ == nullptr)
    {
        image_mutex_.unlock();
        return false;
    }
    else
    {
        cv_image = current_img_ptr_->image;
        image_mutex_.unlock();
        return true;
    }
}


void ImageConverter::imageAcquisitionCallback(const sensor_msgs::ImageConstPtr& msg)
{
    // Copy the image from the sensor_msg and store it internally

    image_mutex_.lock();

    try
    {
        current_img_ptr_ = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

    }
    catch (cv_bridge::Exception& exc)
    {
        ROS_ERROR("Caught cv_bridge exception: %s", exc.what());
    }

    image_mutex_.unlock();

    // if (show_debug_window_)
    // {
    //     cv::imshow(debug_window_name_, current_img_ptr_->image);
    //     cv::waitKey(3);
    // }

    return;
}
