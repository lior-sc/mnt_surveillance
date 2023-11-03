#include "mnt_surveillance_master_node/master_node.hpp"

using mnt_surveillance::master_node::MasterNode;
using mnt_surveillance::master_node::img_decoder::ImgDecoder;

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
    // declare all parameters present in yaml file
    node_handle_->declare_parameter<int>("frame_width_px", 9);
    node_handle_->declare_parameter<int>("frame_height_px", 9);

    node_handle_->declare_parameter<std::string>("encoded_video_topic_name", "/video/raw_data");
    node_handle_->declare_parameter<std::string>("decoded_video_topic_name", "/video/decoded_data");
    node_handle_->declare_parameter<std::string>("alarm_service_name", "/mnt_alarm_service");
    node_handle_->declare_parameter<std::string>("analyzer_video_topic_name", "/video/decoded_data");

    node_handle_->declare_parameter<bool>("alarm_over_saturation_flag",true);
    node_handle_->declare_parameter<double>("alarm_over_saturation_thresold",0.20);
    node_handle_->declare_parameter<int>("alarm_over_saturation_ratio_thresold",1022);

    node_handle_->declare_parameter<bool>("alarm_under_saturation_flag",true);
    node_handle_->declare_parameter<double>("alarm_under_saturation_thresold",0.81);
    node_handle_->declare_parameter<int>("alarm_under_saturation_ratio_thresold",100);

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
    std::string encoded_video_topic_name;
    std::string decoded_video_topic_name;

    node_handle_->get_parameter_or("encoded_video_topic_name",
                                    encoded_video_topic_name,
                                    std::string("/video/raw_data"));

    node_handle_->get_parameter_or("decoded_video_topic_name",
                                    decoded_video_topic_name,
                                    std::string("/video/decoded_data"));    
    img_decoder_object_ = std::make_shared<ImgDecoder>(node_handle_,
                                                        encoded_video_topic_name,
                                                        decoded_video_topic_name);
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