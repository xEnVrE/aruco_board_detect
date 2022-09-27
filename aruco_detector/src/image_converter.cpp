#include <image_converter.h>

#include <sensor_msgs/image_encodings.h>

#include <opencv2/opencv.hpp>


ImageConverter::ImageConverter(ros::NodeHandle& nh) : it_(nh)
{
    // Subscribe to the camera image topic

    image_sub_ = it_.subscribe("input/image_raw", 1, &ImageConverter::imageAcquisitionCallback, this);
}


ImageConverter::~ImageConverter()
{}


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

    return;
}
