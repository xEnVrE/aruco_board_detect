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
#include <opencv2/aruco/dictionary.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <mutex>
#include <memory>

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

class CameraParameters
{
    // this class is supposed to hold camera parameters from a cameraInfo msg

    cv::Mat camera_matrix_;
    cv::Mat distortion_coeffs_;
    cv::Size image_size_;

    bool camera_info_stored_;

public:

    CameraParameters();

    cv::Mat getCameraMatrix();
    cv::Mat getDistortionCoeffs();
    cv::Size getImageSize();

    bool isCamInfoStored();

    void setCameraParameters(const sensor_msgs::CameraInfo& cam_info_msg);

};

class ArucoDetectNode
{
    // this class has an image converter sets up callbacks and calls the board detector once in a while
    // also publishes a tf pose with the board pose
    // optionally shows a debug image

    ros::NodeHandle nh_;

    std::unique_ptr<CameraParameters> cam_params_;

    std::unique_ptr<ImageConverter> img_converter_;

    ros::Subscriber cam_info_sub_;

    ros::Publisher board_pose_pub_;

    ros::Timer timer_;
    float time_between_callbacks_;

    cv::Mat input_img_;
    cv::Mat output_img_;

    std::string debug_window_name_;

    struct ArucoBoardDescription
    {
        int n_markers_x_;
        int n_markers_y_;
        float marker_size_;
        float marker_stride_;
        int dict_type_;
    };

    ArucoBoardDescription board_description_;
    cv::Ptr<cv::aruco::Dictionary> aruco_dict_;
    cv::Ptr<cv::aruco::GridBoard> aruco_board_;









public:

    ArucoDetectNode(ros::NodeHandle& nh);

    void cameraParamsAcquisitionCallback(const sensor_msgs::CameraInfo& cam_info_msg);

    void boardDetectionTimedCallback(const ros::TimerEvent&);


};

