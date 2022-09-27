#ifndef ARUCO_DETECTOR_NODE_H
#define ARUCO_DETECTOR_NODE_H

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/aruco/dictionary.hpp>

#include <memory>
#include <string>

#include <camera_parameters.h>
#include <image_converter.h>


/**
 * @brief Class to perform aruco detection from camera images. It sets up
 * the proper callbacks and call the detector once in a while.
 * Publishes the pose as a tf transform and on a PoseStamped topic.
 *
 */
class ArucoDetectorNode
{
public:
    ArucoDetectorNode(ros::NodeHandle& nh);

    ~ArucoDetectorNode();

    void cameraParamsAcquisitionCallback(const sensor_msgs::CameraInfo& cam_info_msg);

    void detectionTimedCallback(const ros::TimerEvent&);

private:
    bool isIdWithinList(const int& id, const std::vector<int>& list);

    /**
     * Marker description
     */
    struct MarkerDescription
    {
        std::vector<int> marker_ids;
        float marker_size;
        int dict_type;
    };

    MarkerDescription description_;
    cv::Ptr<cv::aruco::Dictionary> aruco_dict_;

    /**
     * ROS-related
     */
    ros::NodeHandle nh_;
    ros::Subscriber cam_info_sub_;
    ros::Publisher pose_pub_;
    ros::Timer timer_;

    /**
     * TF-related
     */
    // tf::TransformBroadcaster board_transform_bc_;

    /**
     * Image handling-related
     */
    image_transport::ImageTransport it_;
    image_transport::Publisher output_image_pub_;
    std::unique_ptr<ImageConverter> img_converter_;
    std::unique_ptr<CameraParameters> cam_params_;
    cv::Mat input_img_;
    cv::Mat output_img_;

    /**
     * Other variables
     */
    bool show_debug_image_;
    float time_between_callbacks_;
    const std::string debug_window_name_ = "ArUco detection";
};

#endif
