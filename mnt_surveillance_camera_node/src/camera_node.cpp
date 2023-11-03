#include "mnt_surveillance_camera_node/camera_node.hpp"

using mnt_surveillance::camera_node::CameraNode;

CameraNode::CameraNode(const std::string &camera_node_name) : Node(camera_node_name)
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
    node_handle_->declare_parameter<int>("frame_width_px", 10);
    node_handle_->declare_parameter<int>("frame_height_px", 10);
    node_handle_->declare_parameter<int>("frame_rate_ms", 32);
    node_handle_->declare_parameter<std::string>("camera_topic_name", "/video/raw_data");    
    node_handle_->declare_parameter<std::string>("camera_type", "webcam");    

}

void CameraNode::add_cameras()
{
    std::string camera_type = node_handle_->get_parameter("camera_type").as_string();
    
    if (camera_type == "webcam")
    {
        RCLCPP_INFO(this->get_logger(), "Adding camera of type: %s", camera_type.c_str());

        encoded_cameras_.push_back(new camera::WebcamEncoded(
        node_handle_,
        node_handle_->get_parameter("camera_topic_name").as_string()));
    }
    else
    {
        RCLCPP_INFO(this->get_logger(), "no cameras were added");
    }
}

void CameraNode::run()
{
    publish_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(node_handle_->get_parameter("frame_rate_ms").as_int()),
        std::bind(&CameraNode::publish_video_streams, this));
}

void CameraNode::publish_video_streams()
{
    // RCLCPP_INFO(this->get_logger(), "Publishing video streams");
    
    for (const auto &camera : encoded_cameras_)
    {
        // camera->publish_capture(); // for testing
        camera->publish_encoded_data();
        // camera->encode_decode_publish_image(); // for testing
    }
}

// CameraNode::CameraNode()