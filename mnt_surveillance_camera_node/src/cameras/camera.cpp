#include "mnt_surveillance_camera_node/cameras/camera.hpp"

using mnt_surveillance::camera_node::camera::Camera;

Camera::Camera(std::shared_ptr<rclcpp::Node> &nh,
               const std::string &topic_name = "")
    : nh_(nh),
      topic_name_(topic_name)
{
  RCLCPP_INFO(nh_->get_logger(), "Init Camera object");

  img_pub_ = nh_->create_publisher<sensor_msgs::msg::Image>(topic_name_, 10);
}

void Camera::publish()
{
  capture();
  img_pub_->publish(*msg_.get());
  RCLCPP_INFO(nh_->get_logger(), "Image published");
}
