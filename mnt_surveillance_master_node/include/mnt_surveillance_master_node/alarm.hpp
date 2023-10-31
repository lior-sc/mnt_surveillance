/**
 * - This is a A listener node that will expose a ROS2 service, once an alarm is triggered a log will
 * be displayed onthe screen.
 */

#ifndef MNT__SURVEILLANCE__ALARM_HPP
#define MNT__SURVEILLANCE__ALARM_HPP

#include <rclcpp/rclcpp.hpp>
#include <example_interfaces/srv/add_two_ints.hpp>

namespace mnt_surveillance
{
    namespace master_node
    {
        namespace alarm
        {
            class Alarm
            {
            public:
                Alarm(std::shared_ptr<rclcpp::Node> &nh,
                      const std::string &service_name);
                ~Alarm() = default;

            private:
                void alarm_service_callback(const example_interfaces::srv::AddTwoInts::Request::SharedPtr request,
                                            example_interfaces::srv::AddTwoInts::Response::SharedPtr response);

                std::shared_ptr<rclcpp::Node> nh_;
                rclcpp::Service<example_interfaces::srv::AddTwoInts>::SharedPtr server_;
            };
        } // namespace alarm
    }     // namespace master_node
} // namespace mnt_surveillance

#endif // MNT__SURVEILLANCE__ALARM_HPP