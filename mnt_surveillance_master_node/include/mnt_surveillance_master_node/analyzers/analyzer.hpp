/**
 * @details
 * Analyzer (CPP)
 * --   The analyzer framework will listen to the video stream and once a frame is ready it will apply
 *      various algorithms on it.
 * --   The applied algorithms should be conﬁgurable from the outside and are provided as 3rd party
 *      either source or precompiled libraries. Please elaborate on how a new algorithm will be
 *      introduced to the system.
 * --   A conﬁguration yaml ﬁle is to be used to conﬁgure which algorithms will be applied and in what
 *      order
 * --   For example one of the algorithms can be over-exposure detection to protect the system from
 *      burglars who will try to apply direct light into the camera to aVoid detection. An alarm will be
 *      sounded if more than 20% of the image is fully saturated.
 *
 * @note Personal notes
 * --   This Class shoud have a subscriber to the video stream and conduct algorithms to it.
 *
 * --    the algorithms should be instroduced as precompiled libraries
 * --   the class will output some data regarding the results of the algoritms
 * --   the class listenes to the /video/raw_data topic
 * --   the class retrieves the img and operates algorithms onto it
 * --   the class derives conclusions and calls a service to set the alarm + deliver a message
 */

#ifndef ANALYZER_HPP
#define ANALYZER_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <example_interfaces/srv/add_two_ints.hpp>
#include <chrono>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>
#include <opencv2/opencv.hpp>

using namespace std::chrono_literals;

namespace mnt_surveillance
{
    namespace master_node
    {
        namespace analyzer
        {
            class Analyzer
            {
            public:
                explicit Analyzer(std::shared_ptr<rclcpp::Node> &nh,
                                  const std::string &img_topic_name,
                                  const std::string &alarm_service_name);
                virtual ~Analyzer() = default;
                void check_alarm_conditions();

            protected:
                virtual void img_sub_callback(const sensor_msgs::msg::Image::SharedPtr msg);
                virtual void create_img_subscriber(const std::string &topic_name);
                virtual void create_alarm_service_client(const std::string &service_name);
                double get_saturated_pixels_ratio(cv::Mat frame, double threshold_value, double max_value);
                double get_dark_pixels_ratio(cv::Mat frame, double threshold_value, double in_value);

                // ros variables
                std::shared_ptr<rclcpp::Node> nh_;
                rclcpp::QoS qos_ = rclcpp::QoS(rclcpp::KeepLast(10));
                rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr img_subscription_;
                sensor_msgs::msg::Image::SharedPtr msg_;

                // openCV variables
                cv::Mat frame_;

                // service variables
                rclcpp::Client<example_interfaces::srv::AddTwoInts>::SharedPtr alarm_client_;
                example_interfaces::srv::AddTwoInts::Request::SharedPtr alarm_request_;
            };

        }

    } // namespace camera
} // namespace mnt_surveillance

#endif // ANALYZER_HPP
