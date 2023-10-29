#include "mnt_surveillance_camera_node/cameras/camera.hpp"

using mnt_surveillance::camera::Camera;

Camera::Camera(std::shared_ptr<rclcpp::Node> &nh, const std::string &frame_id = "")
    : nh_(nh),
      frame_id_(frame_id)
{
  img_pub_ = nh_->create_publisher<sensor_msgs::msg::Image>("img_topic_1", 10);
  img_pub_timer_ = nh->create_wall_timer(33ms, std::bind(&Camera::img_pub_callback, this));
}

void Camera::img_pub_callback()
{
  cv::Mat my_img(cv::Size(640, 480), CV_8UC3);
  cv::randu(my_img, cv::Scalar(0, 0, 0), cv::Scalar(255, 255, 255));
  msg_ = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", my_img).toImageMsg();
  img_pub_->publish(*msg_.get());

  RCLCPP_INFO(nh_->get_logger(), "image published");
}