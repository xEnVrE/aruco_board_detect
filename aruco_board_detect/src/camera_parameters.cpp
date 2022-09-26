#include <camera_parameters.h>

#include <ros/ros.h>


CameraParameters::CameraParameters()
{
    camera_info_stored_ = false;

    camera_matrix_ = cv::Mat(3, 3, CV_64FC1);
    distortion_coeffs_ = cv::Mat(5, 1, CV_64FC1);

    return;
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


std::string CameraParameters::getCameraFrameId()
{
    return camera_frame_id_;
}


bool CameraParameters::isCamInfoStored()
{
    return camera_info_stored_;
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

    // Set image size

    image_size_.height = cam_info_msg.height;
    image_size_.width = cam_info_msg.width;

    // Set the frame_id the image is rooted on

    camera_frame_id_ = static_cast<std::string>(cam_info_msg.header.frame_id);

    camera_info_stored_ = true;
}
