#include "mnt_surveillance_camera_node/camera_node.hpp"

using mnt_surveillance::camera_node::CameraNode;

CameraNode::CameraNode() : Node("camera_node")
{
    RCLCPP_INFO(this->get_logger(), "Init surveillance camera node");
    // get current node handle
    node_handle_ = rclcpp::Node::SharedPtr(this);
    add_cameras();
    run();
}

void CameraNode::add_cameras()
{
    // change this to accept from yaml configuration file
    cameras_.push_back(new camera::Webcam(
        node_handle_,
        "hello1"));
    cameras_.push_back(new camera::RandomNoise(
        node_handle_,
        "hello2"));
}

void CameraNode::run()
{
    publish_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(32),
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