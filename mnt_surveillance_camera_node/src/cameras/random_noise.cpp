#include "mnt_surveillance_camera_node/cameras/random_noise.hpp"

using mnt_surveillance::camera_node::camera::RandomNoise;

RandomNoise::RandomNoise(
    std::shared_ptr<rclcpp::Node> &nh,
    const std::string &topic_name)
    : Camera(nh, topic_name)
{
    RCLCPP_INFO(nh_->get_logger(), "Succeeded to create random noise object");
    // hello
}

bool RandomNoise::capture()
{
    cv::Mat my_img(cv::Size(640, 480), CV_16UC3);
    cv::randu(my_img, cv::Scalar(0, 0, 0), cv::Scalar(255, 255, 255));
    cv::Mat proc_img = this->process_image(my_img);

    msg_ = cv_bridge::CvImage(std_msgs::msg::Header(), "mono16", proc_img).toImageMsg();

    return true;
}

bool RandomNoise::open()
{
    // do nothing
    return true;
}

bool RandomNoise::close()
{
    // do nothing
    return true;
}