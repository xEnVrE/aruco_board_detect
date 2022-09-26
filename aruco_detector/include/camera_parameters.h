#ifndef CAMERA_PARAMETERS_H
#define CAMERA_PARAMETERS_H

#include <sensor_msgs/CameraInfo.h>

#include <opencv2/opencv.hpp>

#include <string>


/**
 * @brief Class to hold camera intrinsic parameters. Parameters will be parsed
 * from a CameraInfo message
 *
 */
class CameraParameters
{
public:
    CameraParameters();

    cv::Mat getCameraMatrix();

    cv::Mat getDistortionCoeffs();

    cv::Size getImageSize();

    std::string getCameraFrameId();

    /**
     * @brief Whether the camera info has already been parsed or not
     *
     * @return true if the info is available
     * @return false otherwise
     */
    bool isCamInfoStored();

    /**
     * @brief Parse the camera parameters
     *
     * @param cam_info_msg ROS message containing the camera info to parse
     */
    void setCameraParameters(const sensor_msgs::CameraInfo& cam_info_msg);

private:
    cv::Mat camera_matrix_;
    cv::Mat distortion_coeffs_;
    cv::Size image_size_;

    std::string camera_frame_id_;

    bool camera_info_stored_;
};

#endif
