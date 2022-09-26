#include <aruco_board_detect_node.h>

#include <cstdlib>
#include <signal.h>



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

    ArucoDetectNode board_detect_node(nh);

    // Set up custom sigint callback
    signal(SIGINT, sigIntHandler);

    ros::spin();

    return EXIT_SUCCESS;
}
