#include "mnt_surveillance_camera_node/cameras/fixed_frame_encoded.hpp"
#include "mnt_surveillance_camera_node/misc/stopwatch.hpp"

using mnt_surveillance::camera_noda::StopWatch;
using mnt_surveillance::camera_node::camera::FixedFrameEncoded;

FixedFrameEncoded::FixedFrameEncoded(
    std::shared_ptr<rclcpp::Node> &nh,
    const std::string &topic_name)
    : CameraEncoded(nh, topic_name)
{
    // create base fixed frame
    this->open();

}

cv::Mat FixedFrameEncoded::capture()
{
    return frame_;
}

bool FixedFrameEncoded::open()
{
    cv::Mat frame(frame_height_px_, frame_width_px_, CV_16UC1);
    RCLCPP_INFO(nh_->get_logger(), "Opening fixed frame with dimensions: %dx%d", frame_width_px_, frame_height_px_);
    for (int i = 0; i < frame.rows; i++) {
        for (int j = 0; j < frame.cols; j++) {
            if ((i + j) % 2 == 0) {
                frame.at<uint16_t>(i, j) = std::numeric_limits<uint16_t>::max();
            } else {
                frame.at<uint16_t>(i, j) = std::numeric_limits<uint16_t>::min();
            }
        }
    }

    // Print the frame data
    std::stringstream ss;
    ss << "Frame data:" << std::endl;
    for (int i = 0; i < frame.rows; i++) {
        for (int j = 0; j < frame.cols; j++) {
            ss << frame.at<uint16_t>(i, j) << " ";
        }
        ss << std::endl;
    }
    RCLCPP_INFO(nh_->get_logger(), "%s", ss.str().c_str());

    frame_ = frame;
    
    RCLCPP_INFO(nh_->get_logger(), "Fixed frame opened");
    return true;
}


bool FixedFrameEncoded::close()
{
    // do nothing
    return true;
}


cv::Mat FixedFrameEncoded::process_image(cv::Mat frame) 
{
    // RCLCPP_INFO(nh_->get_logger(), "hello");
    return frame;
}