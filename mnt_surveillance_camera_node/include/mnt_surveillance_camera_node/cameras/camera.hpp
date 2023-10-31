#ifndef CAMERA_HPP
#define CAMERA_HPP

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
    namespace camera_node
    {
        namespace camera
        {
            class Camera
            {
            public:
                explicit Camera(std::shared_ptr<rclcpp::Node> &nh,
                                const std::string &topic_name_);
                virtual ~Camera() = default;

                virtual bool open() = 0;
                virtual bool close() = 0;
                virtual bool capture() = 0;
                virtual cv::Mat process_image(cv::Mat);
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

#endif // CAMERA_HPP
