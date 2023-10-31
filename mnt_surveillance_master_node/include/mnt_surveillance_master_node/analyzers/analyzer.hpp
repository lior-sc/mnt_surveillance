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
#include <std_msgs/msg/header.hpp>
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
                explicit Analyzer(std::shared_ptr<rclcpp::Node> &nh, const std::string &topic_name_);
                virtual ~Analyzer() = default;

                virtual cv::Mat get_image() = 0;
                virtual void analyze_image(cv::Mat);
                virtual void publish();

            protected:
                void img_pub_callback();

                // ros variables
                std::shared_ptr<rclcpp::Node> nh_;
                std::string frame_id_;
                std::string topic_name_;
                rclcpp::QoS qos_ = rclcpp::QoS(rclcpp::KeepLast(10));
                rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr img_pub_;
                sensor_msgs::msg::Image::SharedPtr msg_;
                rclcpp::TimerBase::SharedPtr img_pub_timer_;

                // openCV variables
                cv::Mat img_;
            };

        }

    } // namespace camera
} // namespace mnt_surveillance

#endif // ANALYZER_HPP
