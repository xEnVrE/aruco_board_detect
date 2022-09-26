#ifndef IMAGE_CONVERTER_H
#define IMAGE_CONVERTER_H

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include <mutex>


/**
 * @brief This class is supposed to catch all camera images
 * from a topic and store them internally.
 *
 */
class ImageConverter
{

public:
    /**
     * @brief Construct a new Image Converter object
     *
     * @param nh ROS node handle. Necessary to subscribe to the input image topic
     * @param show_debug_img Whether to show the received images in a CV window
     */
    ImageConverter(ros::NodeHandle& nh, bool show_debug_img);

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

private:
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    image_transport::Publisher image_pub_;

    cv_bridge::CvImagePtr current_img_ptr_;

    std::mutex image_mutex_;

    bool show_debug_window_;
    std::string debug_window_name_;
};

#endif
