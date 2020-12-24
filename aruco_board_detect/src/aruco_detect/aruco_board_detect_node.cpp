#include <stdio.h>
#include <ros/ros.h>
#include <signal.h>

#include "aruco_board_detect/aruco_board_detect_node.h"

ImageConverter::ImageConverter(ros::NodeHandle& nh, bool show_debug_img) : it_(nh), show_debug_window_(show_debug_img)
{
    // Subscribe to the camera image topic

    image_sub_ = it_.subscribe("input/image_raw", 1, &ImageConverter::imageAcquisitionCallback, this);

    debug_window_name_ = "Input camera image";

    if (show_debug_window_)
        cv::namedWindow(debug_window_name_);

}

ImageConverter::~ImageConverter()
{
    if (show_debug_window_)
        cv::destroyWindow(debug_window_name_);
}

bool ImageConverter::getCurrentImage(cv::Mat& cv_image)
{

    image_mutex_.lock();

    if (current_img_ptr_ == nullptr)
    {
        image_mutex_.unlock();
        return false;
    }
    else
    {
        cv_image = current_img_ptr_->image;
        image_mutex_.unlock();
        return true;
    }
}

void ImageConverter::imageAcquisitionCallback(const sensor_msgs::ImageConstPtr& msg)
{
    // Copy the image from the sensor_msg and store it internally

    image_mutex_.lock();

    try
    {
        current_img_ptr_ = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

    }
    catch (cv_bridge::Exception& exc)
    {
        ROS_ERROR("Caught cv_bridge exception: %s", exc.what());
    }

    image_mutex_.unlock();

    if (show_debug_window_)
    {
        cv::imshow(debug_window_name_, current_img_ptr_->image);
        cv::waitKey(3);
    }

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

    // Set image size

    image_size_.height = cam_info_msg.height;
    image_size_.width = cam_info_msg.width;

    // Set the frame_id the image is rooted on

    camera_frame_id_ = static_cast<std::string>(cam_info_msg.header.frame_id);

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

std::string CameraParameters::getCameraFrameId()
{
    return camera_frame_id_;
}

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
    // img_converter_ = std::make_unique<ImageConverter>(nh);

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

    nh.param<int>("number_markers_x", board_description_.n_markers_x_, 7);
    nh.param<int>("number_markers_y", board_description_.n_markers_y_, 5);
    nh.param<float>("marker_edge_size", board_description_.marker_size_, 0.057);
    nh.param<float>("marker_stride", board_description_.marker_stride_, 0.02);
    nh.param<int>("dictionary_type", board_description_.dict_type_, cv::aruco::DICT_4X4_50);

    aruco_dict_ = cv::aruco::getPredefinedDictionary(board_description_.dict_type_);
    aruco_board_ = cv::aruco::GridBoard::create(board_description_.n_markers_x_,
                                                board_description_.n_markers_y_,
                                                board_description_.marker_size_,
                                                board_description_.marker_stride_,
                                                aruco_dict_);

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

    cv::aruco::detectMarkers(input_img_, aruco_dict_, marker_corners, marker_ids);

    // Using Vec3d causes the estimatePoseBoard method to use them as initial guess in opencv3.2
    // resulting in random estimation
    // Therefore use cv::Mat
    // cv::Vec3d board_rotation, board_position;

    cv::Mat board_rotation, board_position;

    // Compute board pose if at least one marker is detected
    if (marker_ids.size() > 0)
    {
        input_img_.copyTo(output_img_);
        cv::aruco::drawDetectedMarkers(output_img_, marker_corners, marker_ids);

        // Attempt to estimate board pose, and if successful draw origin and axes
        // rotation and position are both assumed to be arrays of 3 numbers

        int board_estimation_outcome = cv::aruco::estimatePoseBoard(marker_corners, marker_ids, aruco_board_,
                                         cam_params_->getCameraMatrix(),
                                         cam_params_->getDistortionCoeffs(),
                                         board_rotation, board_position
                                        );
        if (board_estimation_outcome)
        {
            cv::aruco::drawAxis(output_img_,
                                cam_params_->getCameraMatrix(),
                                cam_params_->getDistortionCoeffs(),
                                board_rotation, board_position, 0.05);

            // Show the output image

            if (show_debug_windows_)
            {
                cv::imshow(debug_window_name_, output_img_);
                cv::waitKey(3);
            }
            // Send the output image

            sensor_msgs::ImagePtr image_msg = cv_bridge::CvImage(std_msgs::Header(),
                                                        "bgr8", output_img_).toImageMsg();
            image_msg->header.frame_id = cam_params_->getCameraFrameId();
            output_image_pub_.publish(image_msg);

            // std::cout << "Board_position: " << std::endl << board_position << std::endl;
            // std::cout << "Board_rotation: " << std::endl << board_rotation << std::endl;

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

            // Take care of single marker messages if necessary

            if (detect_single_markers_)
            {
                std::vector<cv::Vec3d> markers_rot, markers_pos;

                ROS_INFO_STREAM("Detecting single markers with " << marker_corners.size() << "corners.");

                cv::aruco::estimatePoseSingleMarkers(marker_corners, board_description_.marker_size_,
                                        cam_params_->getCameraMatrix(),
                                        cam_params_->getDistortionCoeffs(),
                                        markers_rot, markers_pos);

                ROS_INFO_STREAM("Single markers detected");

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
                    marker_id.data = marker_ids[idx];
                    marker_list_msg->marker_ids.push_back(marker_id);

                }

                // Publish marker data

                markers_data_pub_.publish(marker_list_msg);

            }


        }
    }
}

void sigIntHandler(int sig)
{
    // Gracefully shut down the node when sigint is received

    ROS_INFO("Board detection node shutting down");
    ros::shutdown();
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "aruco_board_detector", ros::init_options::NoSigintHandler);
    ros::NodeHandle nh("~");

    // ImageConverter image_converter(nh);
    ArucoDetectNode board_detect_node(nh);

    // Set up custom sigint callback
    signal(SIGINT, sigIntHandler);

    ros::spin();

    return 0;
}
