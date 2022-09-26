#ifndef ARUCO_DETECTOR_NODE_H
#define ARUCO_DETECTOR_NODE_H

/* ROS */
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

/* OpenCV */
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/aruco/dictionary.hpp>
// #include <opencv2/highgui/highgui.hpp>

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
    /**
     * Marker description
     */
    struct SingleMarkerDescription
    {
        std::vector<int> marker_ids_;
        float marker_size_;
        int dict_type_;
    };

    SingleMarkerDescription single_markers_description_;
    cv::Ptr<cv::aruco::Dictionary> aruco_dict_;

    /**
     * Board description
     */
    // struct ArucoBoardDescription
    // {
    //     int n_markers_x_;
    //     int n_markers_y_;
    //     float marker_size_;
    //     float marker_stride_;
    //     int dict_type_;
    // };
    // ArucoBoardDescription board_description_;
    // cv::Ptr<cv::aruco::GridBoard> aruco_board_;

    /**
     * ROS-related
     */
    ros::NodeHandle nh_;
    ros::Subscriber cam_info_sub_;
    // ros::Publisher board_pose_pub_;
    ros::Publisher markers_data_pub_ ;
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
    bool show_debug_windows_;
    bool detect_single_markers_;
    float time_between_callbacks_;
    const std::string debug_window_name_ = "ArUco detection";
};

#endif
