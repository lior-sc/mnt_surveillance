#include "mnt_surveillance_camera_node/camera_node.hpp"
#include "mnt_surveillance_camera_node/cameras/webcam.hpp"

using mnt_surveillance::camera_node::CameraNode;

CameraNode::CameraNode() : Node("camera_node")
{
    RCLCPP_INFO(this->get_logger(), "Init surveillance camera node");
    node_handle_ = shared_from_this();

    add_cameras();
    run();
}

void CameraNode::add_cameras()
{
    // change this to accept from yaml configuration file
    // cameras_.push_back(Webcam(node_handle_, "webcam_1"));
}

void CameraNode::run()
{
    publish_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(33),
        std::bind(&CameraNode::publish_video_streams, this));
}

void CameraNode::publish_video_streams()
{
    // for (auto &camera : cameras_)
    // {
    //     camera.publish();
    // }
}

// CameraNode::CameraNode()