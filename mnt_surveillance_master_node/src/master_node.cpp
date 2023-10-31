#include "mnt_surveillance_master_node/master_node.hpp"

using mnt_surveillance::master_node::MasterNode;

MasterNode::MasterNode() : Node("master_node")
{
    RCLCPP_INFO(this->get_logger(), "Init master node");
    node_handle_ = rclcpp::Node::SharedPtr(this);
}

void MasterNode::add_analyzers()
{
}
void MasterNode::add_recorders()
{
}
void MasterNode::add_alarms()
{
}
void MasterNode::run()
{
}