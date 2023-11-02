#include "mnt_surveillance_camera_node/camera_node.hpp"

using mnt_surveillance::camera_node::CameraNode;

CameraNode::CameraNode() : Node("camera_node")
{
    RCLCPP_INFO(this->get_logger(), "Init surveillance camera node");
    // get current node handle
    node_handle_ = rclcpp::Node::SharedPtr(this);
    declare_parameters();
    add_cameras();
    run();
}

void CameraNode::declare_parameters()
{
    // change this to accept from yaml configuration file
    node_handle_->declare_parameter<int>("frame_width_px", 50);
    node_handle_->declare_parameter<int>("frame_height_px", 50);
    node_handle_->declare_parameter<int>("frame_rate", 30);
}

void CameraNode::add_cameras()
{
    // change this to accept from yaml configuration file
    // cameras_.push_back(new camera::Webcam(
    //     node_handle_,
    //     "camera/webcam"));
    cameras_.push_back(new camera::RandomNoise(
        node_handle_,
        "camera/random_noise"));
    encoded_cameras_.push_back(new camera::WebcamEncoded(
        node_handle_,
        "webcam_encoded"));
    encoded_cameras_.push_back(new camera::FixedFrameEncoded(
        node_handle_,
        "fixed_frame_encoded"));
}

void CameraNode::run()
{
    publish_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(32),
        std::bind(&CameraNode::publish_video_streams, this));
}

void CameraNode::publish_video_streams()
{
    for (const auto &camera : cameras_)
    {
        camera->publish();
    }

    for (const auto &camera : encoded_cameras_)
    {
        camera->publish_capture();
        camera->publish_encoded_data();
        // camera->publish_decoded_image();
    }
}

// CameraNode::CameraNode()