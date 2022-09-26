#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>

#include <aruco_detector_node.h>
#include <aruco_detector/MarkerList.h>


ArucoDetectorNode::ArucoDetectorNode(ros::NodeHandle& nh) : nh_(nh), time_between_callbacks_(0.2), it_(nh)
{
    // Load parameters
    nh.param<bool>("detect_single_markers", detect_single_markers_, false);
    nh.param<bool>("debug_img", show_debug_windows_, false);
    nh.param<int>("single_markers_config/dictionary_type", single_markers_description_.dict_type_, cv::aruco::DICT_4X4_50);
    nh.param<float>("single_markers_config/marker_edge_size", single_markers_description_.marker_size_, 0.04);
    nh.param<float>("detection_rate", time_between_callbacks_, 1.0);
    nh.param<std::vector<int>>("single_markers_config/marker_ids", single_markers_description_.marker_ids_, std::vector<int>());

    // Setup an instance of ImageConverter
    img_converter_ = std::make_unique<ImageConverter>(nh_);

    // Subscribe to camera parameters topic
    cam_params_ = std::make_unique<CameraParameters>();
    cam_info_sub_ = nh_.subscribe("input/camera_info", 1, &ArucoDetectorNode::cameraParamsAcquisitionCallback, this);

    // Publish output images
    output_image_pub_ = it_.advertise("debug_image", 1);
    // board_pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("board_pose", 1);

    // if (detect_single_markers_)
    //     markers_data_pub_ = nh_.advertise<aruco_board_detect::MarkerList>("markers_data", 1);

    // Setup the detection callback
    timer_ = nh_.createTimer(ros::Duration(time_between_callbacks_), &ArucoDetectorNode::detectionTimedCallback, this);

    // Setup the debugging OpenCV window if required
    if (show_debug_windows_)
        cv::namedWindow(debug_window_name_);
}


ArucoDetectorNode::~ArucoDetectorNode()
{
    // If the debugging window was active, kill the window
    if (show_debug_windows_)
        cv::destroyWindow(debug_window_name_);
}


void ArucoDetectorNode::cameraParamsAcquisitionCallback(const sensor_msgs::CameraInfo& cam_info_msg)
{
    cam_params_->setCameraParameters(cam_info_msg);

    // Once the data has been acquired, remove the subscription
    cam_info_sub_.shutdown();

    ROS_INFO_STREAM("Camera image size: " << cam_params_->getImageSize().width
                    << " x " << cam_params_->getImageSize().height);

    ROS_INFO_STREAM("Camera parameters:" << std::endl
                    << cam_params_->getCameraMatrix());

    ROS_INFO_STREAM("Camera distortion parameters:" << std::endl
                    << cam_params_->getDistortionCoeffs());
}


void ArucoDetectorNode::detectionTimedCallback(const ros::TimerEvent&)
{
    // Camera parameters not yet received
    if (!cam_params_->isCamInfoStored())
    {
        ROS_INFO_STREAM("Camera info has not been parsed yet");
        return;
    }

    // Camera images not yet received
    if (!img_converter_->getCurrentImage(input_img_))
    {
        ROS_INFO_STREAM("No image has been acquired yet");
        return;
    }

    // Perform markers detection
    // std::vector<int> ids;
    // std::vector<std::vector<cv::Point2f>> inliers;
    // detectMarkers(image, get_dictionary(), inliers, ids);

    // if (single_marker_ids.size())
    // {

    //     cv::aruco::estimatePoseSingleMarkers(single_marker_corners, single_markers_description_.marker_size_,
    //                             cam_params_->getCameraMatrix(),
    //                             cam_params_->getDistortionCoeffs(),
    //                             markers_rot, markers_pos);

    //     // Prepare the marker list array message including every detected marker

    //     aruco_board_detect::MarkerListPtr marker_list_msg(new aruco_board_detect::MarkerList);

    //     marker_list_msg->header.frame_id = cam_params_->getCameraFrameId();
    //     marker_list_msg->header.stamp = ros::Time::now();
    //     marker_list_msg->marker_dictionary.data = board_description_.dict_type_;

    //     for (int idx=0; idx < markers_rot.size(); idx++)
    //     {
    //         // Rotation axis to rotation matrix

    //         cv::Mat rot_mat_marker(3, 3, cv::DataType<float>::type);
    //         cv::Rodrigues(markers_rot[idx], rot_mat_marker);

    //         // Rotation matrix to quaternion

    //         tf::Matrix3x3 rot_mat_marker_tf(rot_mat_marker.at<double>(0,0), rot_mat_marker.at<double>(0,1), rot_mat_marker.at<double>(0,2),
    //                             rot_mat_marker.at<double>(1,0), rot_mat_marker.at<double>(1,1), rot_mat_marker.at<double>(1,2),
    //                             rot_mat_marker.at<double>(2,0), rot_mat_marker.at<double>(2,1), rot_mat_marker.at<double>(2,2));

    //         tf::Quaternion quat_marker_tf;
    //         rot_mat_marker_tf.getRotation(quat_marker_tf);

    //         // Push back pose and index

    //         geometry_msgs::Pose marker_pose;
    //         marker_pose.orientation.x = quat_marker_tf.getX();
    //         marker_pose.orientation.y = quat_marker_tf.getY();
    //         marker_pose.orientation.z = quat_marker_tf.getZ();
    //         marker_pose.orientation.w = quat_marker_tf.getW();
    //         marker_pose.position.x = markers_pos[idx][0];
    //         marker_pose.position.y = markers_pos[idx][1];
    //         marker_pose.position.z = markers_pos[idx][2];
    //         marker_list_msg->marker_poses.push_back(marker_pose);

    //         std_msgs::Int16 marker_id;
    //         marker_id.data = single_marker_ids[idx];
    //         marker_list_msg->marker_ids.push_back(marker_id);

    //         // Draw the single markers and axes

    //         cv::aruco::drawAxis(output_img_,
    //                             cam_params_->getCameraMatrix(),
    //                             cam_params_->getDistortionCoeffs(),
    //                             markers_rot[idx],
    //                             markers_pos[idx],
    //                             0.1);

    //     }

    //     cv::aruco::drawDetectedMarkers(output_img_,
    //                         single_marker_corners,
    //                         single_marker_ids,
    //                         cv::Scalar(0,165,255));

    //     // Publish marker data

    //     markers_data_pub_.publish(marker_list_msg);
    // }

    // Send the output image

    // sensor_msgs::ImagePtr image_msg = cv_bridge::CvImage(std_msgs::Header(),
    //                                                 "bgr8", output_img_).toImageMsg();
    // image_msg->header.frame_id = cam_params_->getCameraFrameId();
    // output_image_pub_.publish(image_msg);

    // // Show the output image

    // if (show_debug_windows_)
    // {
    //     cv::imshow(debug_window_name_, output_img_);
    //     cv::waitKey(3);
    // }
}
