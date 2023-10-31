#ifndef MASTER__NODE_HPP_
#define MASTER__NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include "mnt_surveillance_master_node/analyzers/analyzer.hpp"
#include "mnt_surveillance_master_node/alarm.hpp"

namespace mnt_surveillance
{
    namespace master_node
    {
        class MasterNode : public rclcpp::Node
        {
        public:
            MasterNode();
            ~MasterNode() = default;

        private:
            void add_analyzers();
            void add_recorders();
            void add_alarms();
            void run();
            rclcpp::Node::SharedPtr node_handle_;
            rclcpp::TimerBase::SharedPtr publish_timer_;
            std::shared_ptr<alarm::Alarm> alarm_service_server_;

            // Other member functions here
        };

    } // namespace camera_node
} // namespace mnt_surveillance

#endif // MASTER__NODE_HPP_