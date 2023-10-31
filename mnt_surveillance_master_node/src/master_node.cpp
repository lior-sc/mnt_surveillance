#include "mnt_surveillance_master_node/master_node.hpp"

using mnt_surveillance::master_node::MasterNode;

MasterNode::MasterNode() : Node("master_node")
{
    RCLCPP_INFO(this->get_logger(), "Init master node");
    node_handle_ = rclcpp::Node::SharedPtr(this);
    add_alarms();
    run();
}

void MasterNode::add_analyzers()
{
}
void MasterNode::add_recorders()
{
}
void MasterNode::add_alarms()
{
    // create alarm service server instance with a given
    alarm_service_server_ = std::make_shared<alarm::Alarm>(node_handle_, "mnt_alarm");
    return;
}
void MasterNode::run()
{
}