/**
 * - This is a A listener node that will expose a ROS2 service, once an alarm is triggered a log will
 * be displayed onthe screen.
 */

#ifndef MNT__SURVEILLANCE__ALARM_HPP
#define MNT__SURVEILLANCE__ALARM_HPP

#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>

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
                void alarm_service_callback(const std_srvs::srv::Trigger::Request::SharedPtr request,
                                            std_srvs::srv::Trigger::Response::SharedPtr response);

                std::shared_ptr<rclcpp::Node> nh_;
                rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr alarm_server_;
            };
        } // namespace alarm
    }     // namespace master_node
} // namespace mnt_surveillance

#endif // MNT__SURVEILLANCE__ALARM_HPP