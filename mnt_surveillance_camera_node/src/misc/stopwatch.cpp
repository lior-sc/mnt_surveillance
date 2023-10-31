#include "mnt_surveillance_camera_node/misc/stopwatch.hpp"

using mnt_surveillance::camera_noda::StopWatch;

StopWatch::StopWatch(std::shared_ptr<rclcpp::Node> &nh) : nh_(nh)
{
    // Get the current time
    start_time_ = nh_->get_clock()->now();
}

StopWatch::~StopWatch()
{
    rclcpp::Time now = nh_->get_clock()->now();

    double elapsed_time = now.seconds() - start_time_.seconds();
    RCLCPP_INFO(nh_->get_logger(), "######## StopWatch() ######## Elapsed time: %f seconds", elapsed_time);
}