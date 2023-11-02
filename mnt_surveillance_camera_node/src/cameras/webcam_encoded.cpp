#include "mnt_surveillance_camera_node/cameras/webcam_encoded.hpp"
#include "mnt_surveillance_camera_node/misc/stopwatch.hpp"

using mnt_surveillance::camera_noda::StopWatch;
using mnt_surveillance::camera_node::camera::WebcamEncoded;

WebcamEncoded::WebcamEncoded(
    std::shared_ptr<rclcpp::Node> &nh,
    const std::string &topic_name)
    : CameraEncoded(nh, topic_name)
{
    this->open();
}

cv::Mat WebcamEncoded::capture()
{
    cv::Mat frame;
    cap->read(frame);

    return frame;
}

bool WebcamEncoded::open()
{
    RCLCPP_INFO(nh_->get_logger(), "opening camera");
    cap = std::make_unique<cv::VideoCapture>(0);

    std::this_thread::sleep_for(std::chrono::seconds(1)); // add this line

    if (!cap->isOpened())
    {
        RCLCPP_ERROR(nh_->get_logger(), "failed to open teh webcam. change camera port number");
        this->close();
        return false;
    }

    RCLCPP_INFO(nh_->get_logger(), "camera works!");
    RCLCPP_INFO(nh_->get_logger(), "Succeeded to create webcam publisher");

    return true;
}

bool WebcamEncoded::close()
{
    // do nothing
    cap->release();
    return true;
}
