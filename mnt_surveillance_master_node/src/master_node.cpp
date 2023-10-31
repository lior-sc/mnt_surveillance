#include "mnt_surveillance_master_node/master_node.hpp"

using mnt_surveillance::master_node::MasterNode;

MasterNode::MasterNode() : Node("master_node")
{
    RCLCPP_INFO(this->get_logger(), "Init master node");
    node_handle_ = rclcpp::Node::SharedPtr(this);

    std::string img_topic_name = "/video/raw_data";
    std::string alarm_service_name = "/mnt_alarm_service";
    add_alarms(alarm_service_name);
    add_analyzers(img_topic_name,
                  alarm_service_name);
    run();
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