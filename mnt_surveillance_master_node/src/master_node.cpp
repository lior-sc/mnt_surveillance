#include "mnt_surveillance_master_node/master_node.hpp"

using mnt_surveillance::master_node::MasterNode;

MasterNode::MasterNode() : Node("master_node")
{
    RCLCPP_INFO(this->get_logger(), "Init master node");
    node_handle_ = rclcpp::Node::SharedPtr(this);

    declare_parameters();
    std::string analyzer_video_topic_name;
    std::string alarm_service_name;

    node_handle_->get_parameter_or("analyzer_video_topic_name",
                                    analyzer_video_topic_name,
                                    std::string("/video/decoded_data"));
    node_handle_->get_parameter_or("alarm_service_name",
                                    alarm_service_name,
                                    std::string("/mnt_alarm_service"));   

    add_video_decoder();
    add_alarms(alarm_service_name);
    add_analyzers(analyzer_video_topic_name,
                  alarm_service_name);
    run();
}

void MasterNode::declare_parameters()
{
    // declare parameters present in yaml file
    node_handle_->declare_parameter("frame_width_px", 9);
    node_handle_->declare_parameter("frame_height_px", 9);
    node_handle_->declare_parameter("encoded_video_topic_name", "/video/raw_data");
    node_handle_->declare_parameter("decoded_video_topic_name", "/video/decoded_data");
    node_handle_->declare_parameter("alarm_service_name", "/mnt_alarm_service");
    node_handle_->declare_parameter("analyzer_video_topic_name", "/video/decoded_data");
}

void MasterNode::add_analyzers(const std::string &img_topic_name,
                               const std::string &alarm_service_name)
{
    analyzer_object_ = std::make_shared<analyzer::Analyzer>(node_handle_,
                                                            img_topic_name,
                                                            alarm_service_name);
}
void MasterNode::add_recorders()
{
}

/** @details
 * this object will initialize a subscriber to the raw data topic
 * and a publisher to the decoded data topic
*/
void MasterNode::add_video_decoder()
{
    img_decoder_object_ = std::make_shared<ImgDecoder>(node_handle_);
}
void MasterNode::add_alarms(const std::string &alarm_service_name)
{
    // create alarm service server instance with a given
    alarm_service_server_ = std::make_shared<alarm::Alarm>(node_handle_, alarm_service_name);
    return;
}
void MasterNode::run()
{
    /**
     * do nothing. this analyzer is triggered by its subscriber
     * This segment of code is just there for readability
     */
}