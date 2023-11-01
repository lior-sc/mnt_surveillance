#include "mnt_surveillance_master_node/alarm.hpp"

using mnt_surveillance::master_node::alarm::Alarm;
using std::placeholders::_1;
using std::placeholders::_2;

Alarm::Alarm(std::shared_ptr<rclcpp::Node> &nh, const std::string &service_name) : nh_(nh)
{
    alarm_server_ = nh_->create_service<std_srvs::srv::Trigger>(
        service_name,
        std::bind(&Alarm::alarm_service_callback, this, _1, _2));

    RCLCPP_INFO(nh_->get_logger(), "Service server created!");
}

void Alarm::alarm_service_callback(const std_srvs::srv::Trigger::Request::SharedPtr request,
                                   std_srvs::srv::Trigger::Response::SharedPtr response)
{
    (void) request;
    RCLCPP_WARN(nh_->get_logger(), "Alarm service called!!!");

    int status = system("ros2 run rqt_console rqt_console");

    if (status != 0)
    {
        response->success = false;
        response->message = "Error launching rqt_console";
        RCLCPP_ERROR(nh_->get_logger(), response->message.c_str());
        return;
    }
    
    response->success = true;
    response->message = "rqt_console launched successfully";
    RCLCPP_INFO(nh_->get_logger(), response->message.c_str());
    return;
}
