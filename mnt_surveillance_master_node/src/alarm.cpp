#include "mnt_surveillance_master_node/alarm.hpp"

using mnt_surveillance::master_node::alarm::Alarm;
using std::placeholders::_1;
using std::placeholders::_2;

Alarm::Alarm(std::shared_ptr<rclcpp::Node> &nh, const std::string &service_name) : nh_(nh)
{
    server_ = nh_->create_service<example_interfaces::srv::AddTwoInts>(
        service_name,
        std::bind(&Alarm::alarm_service_callback, this, _1, _2));

    RCLCPP_INFO(nh_->get_logger(), "Service server created!");
}

void Alarm::alarm_service_callback(const example_interfaces::srv::AddTwoInts::Request::SharedPtr request,
                                   example_interfaces::srv::AddTwoInts::Response::SharedPtr response)
{
    RCLCPP_WARN(nh_->get_logger(), "Service called!!");
    response->sum = request->a + request->b;
    // do nothing
}
