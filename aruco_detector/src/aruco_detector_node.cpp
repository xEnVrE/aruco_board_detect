#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>

#include <aruco_detector_node.h>
#include <aruco_detector/MarkerList.h>

#include <Eigen/Dense>
#include <opencv2/core/eigen.hpp>


ArucoDetectorNode::ArucoDetectorNode(ros::NodeHandle& nh) : nh_(nh), time_between_callbacks_(0.2), it_(nh)
{
    // Load parameters
    nh.param<bool>("show_debug_image", show_debug_image_, false);
    nh.param<float>("detection_period", time_between_callbacks_, 1.0);

    nh.param<int>("aruco_detector_markers_config/dictionary_type", description_.dict_type, cv::aruco::DICT_4X4_50);
    nh.param<float>("aruco_detector_markers_config/marker_edge_size", description_.marker_size, 0.04);
    nh.param<std::vector<int>>("aruco_detector_markers_config/marker_ids", description_.marker_ids, std::vector<int>());

    // Setup the dictionary
    aruco_dict_ = cv::aruco::getPredefinedDictionary(description_.dict_type);

    // Setup an instance of ImageConverter
    img_converter_ = std::make_unique<ImageConverter>(nh_);

    // Subscribe to camera parameters topic
    cam_params_ = std::make_unique<CameraParameters>();
    cam_info_sub_ = nh_.subscribe("input/camera_info", 1, &ArucoDetectorNode::cameraParamsAcquisitionCallback, this);

    // Publish output images
    output_image_pub_ = it_.advertise("debug_image", 1);

    // Publish output poses
    pose_pub_ = nh_.advertise<aruco_detector::MarkerList>("marker_pose", 1);

    // Setup the detection callback
    timer_ = nh_.createTimer(ros::Duration(time_between_callbacks_), &ArucoDetectorNode::detectionTimedCallback, this);

    // Setup the debugging OpenCV window if required
    if (show_debug_image_)
        cv::namedWindow(debug_window_name_);
}


ArucoDetectorNode::~ArucoDetectorNode()
{
    // If the debugging window was active, kill the window
    if (show_debug_image_)
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
        return;

    const auto& now = ros::Time::now();

    // Prepare an image for output
    input_img_.copyTo(output_img_);

    // Prepare marker list message for output
    aruco_detector::MarkerListPtr marker_list_msg = std::make_unique<aruco_detector::MarkerList>();
    marker_list_msg->header.frame_id = cam_params_->getCameraFrameId();
    marker_list_msg->header.stamp = now;
    marker_list_msg->marker_dictionary.data = description_.dict_type;

    // Perform markers detection
    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f>> inliers;
    cv::aruco::detectMarkers(input_img_, aruco_dict_, inliers, ids);

    if (ids.size() > 0)
    {
        // Perform marker pose estimation.
        std::vector<cv::Vec3d> position;
        std::vector<cv::Vec3d> orientation;
        cv::aruco::estimatePoseSingleMarkers
        (
            inliers, description_.marker_size,
            cam_params_->getCameraMatrix(), cam_params_->getDistortionCoeffs(),
            orientation, position
        );

        // Process markers within the list specified by the user
        for (std::size_t i = 0; i < ids.size(); i++)
        {
            // Draw marker detection and axes
            if (isIdWithinList(ids.at(i), description_.marker_ids))
            {
                const std::vector<int> single_id_list{ids.at(i)};
                const std::vector<std::vector<cv::Point2f>> single_id_inliers{inliers.at(i)};
                cv::aruco::drawDetectedMarkers(output_img_, single_id_inliers, single_id_list);
                cv::aruco::drawAxis
                (
                    output_img_,
                    cam_params_->getCameraMatrix(), cam_params_->getDistortionCoeffs(),
                    orientation.at(i), position.at(i),
                    0.04
                );
            }

            // Compose the marker id
            std_msgs::Int16 marker_id;
            marker_id.data = ids.at(i);

            // Convert rotation vector to a quaternion
            Eigen::Matrix3d rotation_matrix_eig;
            cv::Mat rotation_matrix;
            cv::Rodrigues(orientation.at(i), rotation_matrix);
            cv::cv2eigen(rotation_matrix, rotation_matrix_eig);
            Eigen::Quaterniond quaternion(rotation_matrix_eig);

            // Compose the estimated pose
            geometry_msgs::PoseStamped marker_pose;
            marker_pose.header.stamp = now;
            marker_pose.header.frame_id = cam_params_->getCameraFrameId();
            marker_pose.pose.position.x = position.at(i)[0];
            marker_pose.pose.position.y = position.at(i)[1];
            marker_pose.pose.position.z = position.at(i)[2];
            marker_pose.pose.orientation.x = quaternion.x();
            marker_pose.pose.orientation.y = quaternion.y();
            marker_pose.pose.orientation.z = quaternion.z();
            marker_pose.pose.orientation.w = quaternion.w();

            // Append id and pose within the list message
            marker_list_msg->marker_ids.push_back(marker_id);
            marker_list_msg->marker_poses.push_back(marker_pose);

            // Broadcast the associated TF transform
            tf::Transform tf_transform;
            tf::Quaternion quaternion_tf(quaternion.x(), quaternion.y(), quaternion.z(), quaternion.w());
            tf_transform.setRotation(quaternion_tf);
            tf_transform.setOrigin(tf::Vector3(position.at(i)[0], position.at(i)[1], position.at(i)[2]));
            tf::StampedTransform stamped_transform(tf_transform, now, cam_params_->getCameraFrameId(), "marker_" + std::to_string(ids.at(i)));
            tf_transform_bc_.sendTransform(stamped_transform);
        }

        // Publish all the marker poses
        pose_pub_.publish(marker_list_msg);
    }

    // Send the output image
    sensor_msgs::ImagePtr image_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", output_img_).toImageMsg();
    image_msg->header.frame_id = cam_params_->getCameraFrameId();
    output_image_pub_.publish(image_msg);

    // Show the output image
    if (show_debug_image_)
    {
        cv::imshow(debug_window_name_, output_img_);
        cv::waitKey(3);
    }
}


bool ArucoDetectorNode::isIdWithinList(const int& id, const std::vector<int>& list)
{
    return std::find(begin(list), end(list), id) != end(list);
}
