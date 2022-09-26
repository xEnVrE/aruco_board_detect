#include <aruco_detector_node.h>

#include <cstdlib>
#include <signal.h>



void sigIntHandler(int sig)
{
    // Gracefully shut down the node when sigint is received
    ROS_INFO("Detector node shutting down.");
    ros::shutdown();
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "aruco_board_detector", ros::init_options::NoSigintHandler);
    ros::NodeHandle nh("~");

    ArucoDetectorNode detector_node(nh);

    // Set up custom sigint callback
    signal(SIGINT, sigIntHandler);

    ros::spin();

    return EXIT_SUCCESS;
}
