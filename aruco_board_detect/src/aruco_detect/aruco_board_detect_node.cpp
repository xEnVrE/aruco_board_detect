#include <stdio.h>
#include <ros/ros.h>
#include <signal.h>

#include "aruco_detect/aruco_board_detect_node.hpp"

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

CameraParameters::CameraParameters()
{
    camera_info_stored_ = false;

    camera_matrix_ = cv::Mat(3, 3, CV_64FC1);
    distortion_coeffs_ = cv::Mat(5, 1, CV_64FC1);

    return;
}

void CameraParameters::setCameraParameters(const sensor_msgs::CameraInfo& cam_info_msg)
{
    // Assume the camera info msg has 5 distortion coefficients

    camera_matrix_.setTo(0);

    for (int idx = 0; idx < 9; ++idx)
        camera_matrix_.at<double>(idx/3, idx%3) = cam_info_msg.K[idx];

    distortion_coeffs_.setTo(0);

    if (!cam_info_msg.distortion_model.compare("plumb_bob"))
    {
        for (int idx = 0; idx < 5; ++idx)
            distortion_coeffs_.at<double>(idx, 0) = cam_info_msg.D[idx];
    }
    else
    {
        ROS_WARN("Distortion model %s not supported, assuming zero distortion.", cam_info_msg.distortion_model.c_str());
    }

    image_size_.height = cam_info_msg.height;
    image_size_.width = cam_info_msg.width;

    camera_info_stored_ = true;

}

cv::Mat CameraParameters::getCameraMatrix()
{
    return camera_matrix_;
}

cv::Mat CameraParameters::getDistortionCoeffs()
{
    return distortion_coeffs_;
}

cv::Size CameraParameters::getImageSize()
{
    return image_size_;
}

bool CameraParameters::isCamInfoStored()
{
    return camera_info_stored_;
}

ArucoDetectNode::ArucoDetectNode(ros::NodeHandle& nh)
{

    img_converter_ = std::unique_ptr<ImageConverter>(new ImageConverter(nh));

    // img_converter_ = std::make_unique<ImageConverter>(nh);

    cam_info_sub_ = nh.subscribe("/camera/color/camera_info", 1, &ArucoDetectNode::cameraParamsAcquisitionCallback, this);

}

void ArucoDetectNode::cameraParamsAcquisitionCallback(const sensor_msgs::CameraInfo& cam_info_msg)
{
    // Set the object fields

    cam_params_.setCameraParameters(cam_info_msg);
    cam_info_sub_.shutdown();

    std::cout << "Camera parameters:" << std::endl;
    std::cout << cam_params_.getCameraMatrix() << std::endl;

    std::cout << "Distortion parameters: " << std::endl;
    std::cout << cam_params_.getDistortionCoeffs() << std::endl;

    ROS_INFO_STREAM("Camera parameters:" << std::endl
                    << cam_params_.getCameraMatrix());

    ROS_INFO_STREAM("Camera distortion parameters:" << std::endl
                    << cam_params_.getDistortionCoeffs());

    ROS_INFO_STREAM("Camera image size: " << cam_params_.getImageSize().width
                    << " x " << cam_params_.getImageSize().height);

}







void sigIntHandler(int sig)
{
    // Gracefully shut down the node when sigint is received

    ROS_INFO("Board detection node shutting down");
    ros::shutdown();
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "aruco_board_detect_node", ros::init_options::NoSigintHandler);
    ros::NodeHandle nh;

    // ImageConverter image_converter(nh);
    ArucoDetectNode board_detect_node(nh);

    // Set up custom sigint callback
    signal(SIGINT, sigIntHandler);

    ros::spin();

    return 0;
}
