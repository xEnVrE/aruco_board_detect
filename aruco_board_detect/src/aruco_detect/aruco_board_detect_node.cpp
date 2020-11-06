#include <stdio.h>
#include <ros/ros.h>
#include <signal.h>

#include "aruco_detect/aruco_board_detect_node.hpp"

ImageConverter::ImageConverter(ros::NodeHandle& nh) : it_(nh)
{
    // Subscribe to the camera image topic

    image_sub_ = it_.subscribe("/camera/color/image_raw", 1, &ImageConverter::imageAcquisitionCallback, this);

    debug_window_name_ = "Input camera image";
    cv::namedWindow(debug_window_name_);

}

ImageConverter::~ImageConverter()
{
    cv::destroyWindow(debug_window_name_);
}

void ImageConverter::getCurrentImage(cv::Mat& cv_image)
{
    // Assign the image field of the current cv_bridge::cvImage

    image_mutex_.lock();

    cv_image = current_img_ptr_->image;

    image_mutex_.unlock();

    return;
}

void ImageConverter::imageAcquisitionCallback(const sensor_msgs::ImageConstPtr& msg)
{
    // Copy the image from the sensor_msg and store it internally

    image_mutex_.lock();

    try
    {
        current_img_ptr_ = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

        cv::imshow(debug_window_name_, current_img_ptr_->image);
        cv::waitKey(3);
    }
    catch (cv_bridge::Exception& exc)
    {
        ROS_ERROR("Caught cv_bridge exception: %s", exc.what());
    }

    image_mutex_.unlock();

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

    image_size_.height = cam_info_msg.height;
    image_size_.width = cam_info_msg.width;

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

ArucoDetectNode::ArucoDetectNode(ros::NodeHandle& nh) : nh_(nh), time_between_callbacks_(0.2)
{

    img_converter_ = std::unique_ptr<ImageConverter>(new ImageConverter(nh_));
    // img_converter_ = std::make_unique<ImageConverter>(nh);

    cam_info_sub_ = nh_.subscribe("/camera/color/camera_info", 1, &ArucoDetectNode::cameraParamsAcquisitionCallback, this);

    board_pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/board_pose", 1);

    // Set up the timer
    // #TODO source time between callbacks from param server
    timer_ = nh_.createTimer(ros::Duration(time_between_callbacks_), &ArucoDetectNode::boardDetectionTimedCallback, this);

    // Set up the board
    // Hardcoded type for now, will get sourced from param server later #TODO
    board_description_.n_markers_x_ = 7;
    board_description_.n_markers_y_ = 5;
    board_description_.marker_size_ = 0.057;
    board_description_.marker_stride_ = 0.02;
    board_description_.dict_type_ = cv::aruco::DICT_4X4_50;

    aruco_dict_ = cv::aruco::getPredefinedDictionary(board_description_.dict_type_);
    aruco_board_ = cv::aruco::GridBoard::create(board_description_.n_markers_x_,
                                                board_description_.n_markers_y_,
                                                board_description_.marker_size_,
                                                board_description_.marker_stride_,
                                                aruco_dict_);

    debug_window_name_ = "Board camera image";
    cv::namedWindow(debug_window_name_);

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
    // Detect markers, compute board pose and publish transform on tf

    ROS_INFO_STREAM("Detecting board pose at time " << ros::Time::now().sec);


    std::vector<int> marker_ids;
    std::vector<std::vector<cv::Point2f>> marker_corners;
    img_converter_->getCurrentImage(input_img_);
    cv::aruco::detectMarkers(input_img_, aruco_dict_, marker_corners, marker_ids);
    // cv::Vec3d board_rotation, board_position;
    cv::Mat board_rotation, board_position;

    // Compute board pose if at least one marker is detected
    if (marker_ids.size() > 0)
    {
        input_img_.copyTo(output_img_);
        cv::aruco::drawDetectedMarkers(output_img_, marker_corners, marker_ids);

        // Attempt to estimate board pose, and if successful draw origin and axes
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

            cv::imshow(debug_window_name_, output_img_);
            cv::waitKey(3);

            // Send board pose on topic

            geometry_msgs::PoseStamped board_pose;
            board_pose.header.stamp = ros::Time::now();
            board_pose.header.frame_id = "world";
            // board_pose.pose.position.x = board_position[0];
            // board_pose.pose.position.y = board_position[1];
            // board_pose.pose.position.z = board_position[2];
            board_pose.pose.position.x = board_position.at<double>(1);
            board_pose.pose.position.y = board_position.at<double>(2);
            board_pose.pose.position.z = board_position.at<double>(3);


            cv::Mat rot_mat(3, 3, cv::DataType<float>::type);
            cv::Rodrigues(board_rotation, rot_mat);
            tf::Matrix3x3 rot_mat_tf(rot_mat.at<double>(0,0), rot_mat.at<double>(0,1), rot_mat.at<double>(0,2),
                                       rot_mat.at<double>(1,0), rot_mat.at<double>(1,1), rot_mat.at<double>(1,2),
                                       rot_mat.at<double>(2,0), rot_mat.at<double>(2,1), rot_mat.at<double>(2,2));
            tf::Quaternion quat_tf;
            rot_mat_tf.getRotation(quat_tf);
            board_pose.pose.orientation.x = quat_tf.getX();
            board_pose.pose.orientation.y = quat_tf.getY();
            board_pose.pose.orientation.z = quat_tf.getZ();
            board_pose.pose.orientation.w = quat_tf.getW();

            board_pose_pub_.publish(board_pose);

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
    ros::init(argc, argv, "aruco_board_detect_node", ros::init_options::NoSigintHandler);
    ros::NodeHandle nh;

    // ImageConverter image_converter(nh);
    ArucoDetectNode board_detect_node(nh);

    // Set up custom sigint callback
    signal(SIGINT, sigIntHandler);

    ros::spin();

    return 0;
}
