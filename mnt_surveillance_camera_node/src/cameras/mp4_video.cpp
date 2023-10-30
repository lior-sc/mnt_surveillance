#include "mnt_surveillance_camera_node/cameras/mp4_video.hpp"

using mnt_surveillance::camera_node::camera::Mp4Video;

Mp4Video::Mp4Video(
    std::shared_ptr<rclcpp::Node> &nh,
    const std::string &topic_name)
    : Camera(nh, topic_name)
{
    RCLCPP_INFO(nh_->get_logger(), "Succeeded to create random noise object");
    // hello
}

bool Mp4Video::capture()
{
    cv::Mat my_img(cv::Size(640, 480), CV_8UC3);
    cv::randu(my_img, cv::Scalar(0, 0, 0), cv::Scalar(255, 255, 255));
    msg_ = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", my_img).toImageMsg();

    return true;
}

bool Mp4Video::open()
{
    // do nothing
    return true;
}

bool Mp4Video::close()
{
    // do nothing
    return true;
}