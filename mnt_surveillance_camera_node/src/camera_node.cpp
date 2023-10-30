#include "mnt_surveillance_camera_node/camera_node.hpp"
#include "mnt_surveillance_camera_node/cameras/webcam.hpp"

using mnt_surveillance::camera_node::CameraNode;

CameraNode::CameraNode() : Node("camera_node")
{
    RCLCPP_INFO(this->get_logger(), "Init surveillance camera node");
    node_handle_ = std::shared_ptr<::rclcpp::Node>(this, [](::rclcpp::Node *) {});
    RCLCPP_INFO(this->get_logger(), "are we there yet?");
    add_cameras();
    run();
}

void CameraNode::add_cameras()
{
    // change this to accept from yaml configuration file
    cameras_.push_back(new camera::Webcam(
        node_handle_,
        "hellllllllllo"));
}

void CameraNode::run()
{
    publish_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(33),
        std::bind(&CameraNode::publish_video_streams, this));
}

void CameraNode::publish_video_streams()
{
    // RCLCPP_INFO(this->get_logger(), "he;;;;llllooooo!");
    for (const auto &camera : cameras_)
    {
        camera->publish();
    }
}

// CameraNode::CameraNode()