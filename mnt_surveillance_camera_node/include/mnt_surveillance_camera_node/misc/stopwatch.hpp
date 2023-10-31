#ifndef STOPWATCH_HPP
#define STOPWATCH_HPP

#include <chrono>
#include <rclcpp/rclcpp.hpp>

namespace mnt_surveillance
{
    namespace camera_noda
    {
        class StopWatch
        {
        public:
            StopWatch(std::shared_ptr<rclcpp::Node> &nh);
            ~StopWatch();

        private:
            std::shared_ptr<rclcpp::Node> nh_;
            rclcpp::Time start_time_;
        };
    }

}

#endif // STOPWATCH_HPP
