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
            std::chrono::time_point<std::chrono::high_resolution_clock> start_time_;
            std::chrono::time_point<std::chrono::high_resolution_clock> end_time_;
            bool running_;
            double elapsed_time() const;
            rclcpp::Time start_time_;
        };
    }

}

#endif // STOPWATCH_HPP
