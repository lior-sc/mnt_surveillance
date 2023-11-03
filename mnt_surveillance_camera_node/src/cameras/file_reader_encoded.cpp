#include "mnt_surveillance_camera_node/cameras/file_reader_encoded.hpp"
#include "mnt_surveillance_camera_node/misc/stopwatch.hpp"

using mnt_surveillance::camera_noda::StopWatch;
using mnt_surveillance::camera_node::camera::FileReaderEncoded;

FileReaderEncoded::FileReaderEncoded(
    std::shared_ptr<rclcpp::Node> &nh,
    const std::string &topic_name)
    : CameraEncoded(nh, topic_name)
{
    this->open();
}

cv::Mat FileReaderEncoded::capture()
{
    cv::Mat frame;
    std::ifstream file(nh_->get_parameter("file_path").as_string(), 
                    std::ios::binary);

    if (!file.is_open())

    return frame;
}

bool FileReaderEncoded::open()
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

bool FileReaderEncoded::close()
{
    // do nothing
    cap->release();
    return true;
}
