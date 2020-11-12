// ROS
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <tf/transform_broadcaster.h>

// OpenCV + ArUCO
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/aruco/dictionary.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <mutex>
#include <memory>


/**
 * @brief This class is supposed to catch all camera images
 * from a topic and store them internally.
 *
 */
class ImageConverter
{

    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    image_transport::Publisher image_pub_;

    cv_bridge::CvImagePtr current_img_ptr_;

    std::mutex image_mutex_;

    std::string debug_window_name_;

public:

    /**
     * @brief Construct a new Image Converter object
     *
     * @param nh ROS node handle. Necessary to subscribe to the input image topic
     */
    ImageConverter(ros::NodeHandle& nh);

    ~ImageConverter();

    /**
     * @brief Get the last image stored
     *
     * @param cv_image The stored image will be copied to this reference
     * @return true if an image is present
     * @return false if no image has been stored yet
     */
    bool getCurrentImage(cv::Mat& cv_image);

    /**
     * @brief Callback to be executed whenever an image is published on the input topic
     *
     * @param msg Pointer to the image message
     */
    void imageAcquisitionCallback(const sensor_msgs::ImageConstPtr& msg);

};

/**
 * @brief Class to hold camera intrinsic parameters. Parameters will be parsed
 * from a CameraInfo message
 *
 */
class CameraParameters
{

    cv::Mat camera_matrix_;
    cv::Mat distortion_coeffs_;
    cv::Size image_size_;

    std::string camera_frame_id_;

    bool camera_info_stored_;

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

};


/**
 * @brief Class to perform aruco board detection from camera images. It sets up
 * the proper callbacks and call the board detector once in a while.
 * Publishes the pose as a tf transform and on a PoseStamped topic.
 *
 */
class ArucoDetectNode
{

    ros::NodeHandle nh_;

    image_transport::ImageTransport it_;

    std::unique_ptr<CameraParameters> cam_params_;

    std::unique_ptr<ImageConverter> img_converter_;

    ros::Subscriber cam_info_sub_;

    ros::Publisher board_pose_pub_;

    image_transport::Publisher output_image_pub_;

    tf::TransformBroadcaster board_transform_bc_;

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

