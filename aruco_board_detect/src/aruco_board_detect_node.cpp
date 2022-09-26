#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>

#include <aruco_board_detect_node.h>
#include <aruco_board_detect/MarkerList.h>


ArucoDetectNode::ArucoDetectNode(ros::NodeHandle& nh) : nh_(nh), time_between_callbacks_(0.2), it_(nh)
{

    // Set up whether we want the debug images to show

    debug_window_name_ = "Board camera image";

    nh.param<bool>("debug_img", show_debug_windows_, false);
    if (show_debug_windows_)
        cv::namedWindow(debug_window_name_);

    // Set up whether we want to detect and publish single marker poses

    nh.param<bool>("detect_single_markers", detect_single_markers_, false);

    img_converter_ = std::unique_ptr<ImageConverter>(new ImageConverter(nh_, show_debug_windows_));

    // Publish output images

    output_image_pub_ = it_.advertise("debug_image", 1);

    cam_params_ = std::unique_ptr<CameraParameters>(new CameraParameters());

    cam_info_sub_ = nh_.subscribe("input/camera_info", 1, &ArucoDetectNode::cameraParamsAcquisitionCallback, this);

    board_pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("board_pose", 1);

    if (detect_single_markers_)
        markers_data_pub_ = nh_.advertise<aruco_board_detect::MarkerList>("markers_data", 1);

    // Set up the timer

    nh.param<float>("detection_rate", time_between_callbacks_, 1.0);
    timer_ = nh_.createTimer(ros::Duration(time_between_callbacks_), &ArucoDetectNode::boardDetectionTimedCallback, this);

    // Set up the board

    nh.param<int>("board_config/number_markers_x", board_description_.n_markers_x_, 7);
    nh.param<int>("board_config/number_markers_y", board_description_.n_markers_y_, 5);
    nh.param<float>("board_config/marker_edge_size", board_description_.marker_size_, 0.057);
    nh.param<float>("board_config/marker_stride", board_description_.marker_stride_, 0.02);
    nh.param<int>("board_config/dictionary_type", board_description_.dict_type_, cv::aruco::DICT_4X4_50);

    aruco_dict_ = cv::aruco::getPredefinedDictionary(board_description_.dict_type_);
    aruco_board_ = cv::aruco::GridBoard::create(board_description_.n_markers_x_,
                                                board_description_.n_markers_y_,
                                                board_description_.marker_size_,
                                                board_description_.marker_stride_,
                                                aruco_dict_);

    // Set up single marker data

    nh.param<float>("single_markers_config/marker_edge_size", single_markers_description_.marker_size_, board_description_.marker_size_);
    nh.param<int>("single_markers_config/dictionary_type", single_markers_description_.dict_type_, cv::aruco::DICT_4X4_50);
    nh.param<std::vector<int>>("single_markers_config/marker_ids", single_markers_description_.marker_ids_, std::vector<int>());

}


ArucoDetectNode::~ArucoDetectNode()
{
    // If debug was active, kill the window

    if (show_debug_windows_)
        cv::destroyWindow(debug_window_name_);

}


void ArucoDetectNode::cameraParamsAcquisitionCallback(const sensor_msgs::CameraInfo& cam_info_msg)
{
    // Set the object fields

    cam_params_ = std::unique_ptr<CameraParameters>(new CameraParameters());
    cam_params_->setCameraParameters(cam_info_msg);
    cam_info_sub_.shutdown();

    std::cout << "Camera parameters:" << std::endl;
    std::cout << cam_params_->getCameraMatrix() << std::endl;

    std::cout << "Distortion parameters: " << std::endl;
    std::cout << cam_params_->getDistortionCoeffs() << std::endl;

    ROS_INFO_STREAM("Camera parameters:" << std::endl
                    << cam_params_->getCameraMatrix());

    ROS_INFO_STREAM("Camera distortion parameters:" << std::endl
                    << cam_params_->getDistortionCoeffs());

    ROS_INFO_STREAM("Camera image size: " << cam_params_->getImageSize().width
                    << " x " << cam_params_->getImageSize().height);

}


void ArucoDetectNode::boardDetectionTimedCallback(const ros::TimerEvent&)
{

    // Node not ready yet: camera parameters have not been parsed
    if (!cam_params_->isCamInfoStored())
    {
        ROS_INFO_STREAM("Camera info has not been parsed yet");
        return;
    }

    if (!img_converter_->getCurrentImage(input_img_))
    {
        ROS_INFO_STREAM("No image has been acquired yet");
        return;
    }

    // Detect markers, compute board pose and publish transform on tf

    ROS_INFO_STREAM("Detecting board pose at time " << ros::Time::now().sec);

    std::vector<int> marker_ids;
    std::vector<std::vector<cv::Point2f>> marker_corners;
    std::vector<cv::Vec3d> marker_rvecs, marker_tvecs;

    cv::aruco::detectMarkers(input_img_, aruco_dict_, marker_corners, marker_ids);

    // Split the vector of detected markers

    std::vector<int> single_marker_ids;
    std::vector<std::vector<cv::Point2f>> single_marker_corners;
    std::vector<int> board_marker_ids;
    std::vector<std::vector<cv::Point2f>> board_marker_corners;

    // Using Vec3d causes the estimatePoseBoard method to use them as initial guess in opencv3.2
    // resulting in random estimation
    // Therefore use cv::Mat
    // cv::Vec3d board_rotation, board_position;

    cv::Mat board_rotation, board_position;

    // Compute board pose if at least one marker is detected
    if (marker_ids.size() > 0)
    {
        // By knowing the number of markers in the board, we can filter out from the estimation the ones whose
        // ID is not within bounds
        // e.g. a 7x5 board has markers with ids ranging from 1 to 35
        // Split the markers that belong to the board and the singles

        for (int detected_marker_idx = 0; detected_marker_idx < marker_ids.size(); detected_marker_idx++)
        {
            if (marker_ids[detected_marker_idx] > 0 && marker_ids[detected_marker_idx] < board_description_.n_markers_x_ * board_description_.n_markers_y_)
            {
                // Marker belongs to board

                board_marker_ids.push_back(marker_ids[detected_marker_idx]);
                board_marker_corners.push_back(marker_corners[detected_marker_idx]);
            }

            if (detect_single_markers_ && single_markers_description_.marker_ids_.size())
            {
                if (std::find(single_markers_description_.marker_ids_.begin(),
                              single_markers_description_.marker_ids_.end(),
                              marker_ids[detected_marker_idx]) != single_markers_description_.marker_ids_.end())
                {
                    // Marker is a single desired marker

                    single_marker_ids.push_back(marker_ids[detected_marker_idx]);
                    single_marker_corners.push_back(marker_corners[detected_marker_idx]);
                }
            }
        }

        // Draw all the markers for maximum debugging
        input_img_.copyTo(output_img_);

        // Attempt to estimate board pose, and if successful draw origin and axes
        // rotation and position are both assumed to be arrays of 3 numbers

        int board_estimation_outcome = cv::aruco::estimatePoseBoard(board_marker_corners, board_marker_ids, aruco_board_,
                                         cam_params_->getCameraMatrix(),
                                         cam_params_->getDistortionCoeffs(),
                                         board_rotation, board_position
                                        );
        if (board_estimation_outcome)
        {
            // std::cout << "Board_position: " << std::endl << board_position << std::endl;
            // std::cout << "Board_rotation: " << std::endl << board_rotation << std::endl;

            ROS_INFO_STREAM("Marker board detected");

            // Obtain a proper rotation matrix from rotation vector

            cv::Mat rot_mat(3, 3, cv::DataType<float>::type);
            cv::Rodrigues(board_rotation, rot_mat);

            // ROS messages need the rotation in quaternion form

            tf::Matrix3x3 rot_mat_tf(rot_mat.at<double>(0,0), rot_mat.at<double>(0,1), rot_mat.at<double>(0,2),
                                       rot_mat.at<double>(1,0), rot_mat.at<double>(1,1), rot_mat.at<double>(1,2),
                                       rot_mat.at<double>(2,0), rot_mat.at<double>(2,1), rot_mat.at<double>(2,2));
            tf::Quaternion quat_tf;
            rot_mat_tf.getRotation(quat_tf);

            // Send board pose on topic

            geometry_msgs::PoseStamped board_pose;
            board_pose.header.stamp = ros::Time::now();
            board_pose.header.frame_id = cam_params_->getCameraFrameId();
            // board_pose.pose.position.x = board_position[0];
            // board_pose.pose.position.y = board_position[1];
            // board_pose.pose.position.z = board_position[2];
            board_pose.pose.position.x = board_position.at<double>(0);
            board_pose.pose.position.y = board_position.at<double>(1);
            board_pose.pose.position.z = board_position.at<double>(2);
            board_pose.pose.orientation.x = quat_tf.getX();
            board_pose.pose.orientation.y = quat_tf.getY();
            board_pose.pose.orientation.z = quat_tf.getZ();
            board_pose.pose.orientation.w = quat_tf.getW();

            board_pose_pub_.publish(board_pose);

            // Broadcast the TF transform of the board wrt the camera frame id

            tf::Transform board_transform;
            board_transform.setRotation(quat_tf);
            board_transform.setOrigin(tf::Vector3(board_position.at<double>(0),
                                                  board_position.at<double>(1),
                                                  board_position.at<double>(2)));
            tf::StampedTransform board_stamped_transform(board_transform,
                                                         ros::Time::now(),
                                                         cam_params_->getCameraFrameId(),
                                                         "aruco_board");
            board_transform_bc_.sendTransform(board_stamped_transform);

            // Compute and broadcast the GRASPA board ref frame
            // Basically move the aruco board ref frame along the x axis

            tf::Transform board_graspa_transform;
            board_graspa_transform.setIdentity();
            board_graspa_transform.setOrigin(tf::Vector3(static_cast<double>(board_description_.n_markers_x_ * board_description_.marker_size_ + (board_description_.n_markers_x_ - 1) * board_description_.marker_stride_), 0.0, 0.0));
            tf::StampedTransform board_graspa_stamped_transform(board_graspa_transform,
                                                                ros::Time::now(),
                                                                "aruco_board",
                                                                "graspa_board");
            board_transform_bc_.sendTransform(board_graspa_stamped_transform);

            // Draw the board ref system and markers

            cv::aruco::drawDetectedMarkers(output_img_,
                                          board_marker_corners,
                                          board_marker_ids,
                                          cv::Scalar(0,255,165));

            cv::aruco::drawAxis(output_img_,
                                cam_params_->getCameraMatrix(),
                                cam_params_->getDistortionCoeffs(),
                                board_rotation, board_position, 0.1);
        }

        // Take care of single marker messages if necessary

        if (detect_single_markers_)
        {
            std::vector<cv::Vec3d> markers_rot, markers_pos;

            ROS_INFO_STREAM("Detected " << single_marker_ids.size() << " single markers");

            if (single_marker_ids.size())
            {

                cv::aruco::estimatePoseSingleMarkers(single_marker_corners, single_markers_description_.marker_size_,
                                        cam_params_->getCameraMatrix(),
                                        cam_params_->getDistortionCoeffs(),
                                        markers_rot, markers_pos);

                // Prepare the marker list array message including every detected marker

                aruco_board_detect::MarkerListPtr marker_list_msg(new aruco_board_detect::MarkerList);

                marker_list_msg->header.frame_id = cam_params_->getCameraFrameId();
                marker_list_msg->header.stamp = ros::Time::now();
                marker_list_msg->marker_dictionary.data = board_description_.dict_type_;

                for (int idx=0; idx < markers_rot.size(); idx++)
                {
                    // Rotation axis to rotation matrix

                    cv::Mat rot_mat_marker(3, 3, cv::DataType<float>::type);
                    cv::Rodrigues(markers_rot[idx], rot_mat_marker);

                    // Rotation matrix to quaternion

                    tf::Matrix3x3 rot_mat_marker_tf(rot_mat_marker.at<double>(0,0), rot_mat_marker.at<double>(0,1), rot_mat_marker.at<double>(0,2),
                                        rot_mat_marker.at<double>(1,0), rot_mat_marker.at<double>(1,1), rot_mat_marker.at<double>(1,2),
                                        rot_mat_marker.at<double>(2,0), rot_mat_marker.at<double>(2,1), rot_mat_marker.at<double>(2,2));

                    tf::Quaternion quat_marker_tf;
                    rot_mat_marker_tf.getRotation(quat_marker_tf);

                    // Push back pose and index

                    geometry_msgs::Pose marker_pose;
                    marker_pose.orientation.x = quat_marker_tf.getX();
                    marker_pose.orientation.y = quat_marker_tf.getY();
                    marker_pose.orientation.z = quat_marker_tf.getZ();
                    marker_pose.orientation.w = quat_marker_tf.getW();
                    marker_pose.position.x = markers_pos[idx][0];
                    marker_pose.position.y = markers_pos[idx][1];
                    marker_pose.position.z = markers_pos[idx][2];
                    marker_list_msg->marker_poses.push_back(marker_pose);

                    std_msgs::Int16 marker_id;
                    marker_id.data = single_marker_ids[idx];
                    marker_list_msg->marker_ids.push_back(marker_id);

                    // Draw the single markers and axes

                    cv::aruco::drawAxis(output_img_,
                                        cam_params_->getCameraMatrix(),
                                        cam_params_->getDistortionCoeffs(),
                                        markers_rot[idx],
                                        markers_pos[idx],
                                        0.1);

                }

                cv::aruco::drawDetectedMarkers(output_img_,
                                    single_marker_corners,
                                    single_marker_ids,
                                    cv::Scalar(0,165,255));

                // Publish marker data

                markers_data_pub_.publish(marker_list_msg);
            }
        }

        // Send the output image

        sensor_msgs::ImagePtr image_msg = cv_bridge::CvImage(std_msgs::Header(),
                                                    "bgr8", output_img_).toImageMsg();
        image_msg->header.frame_id = cam_params_->getCameraFrameId();
        output_image_pub_.publish(image_msg);

        // Show the output image

        if (show_debug_windows_)
        {
            cv::imshow(debug_window_name_, output_img_);
            cv::waitKey(3);
        }
    }
}
