#ifndef CAMERA__ENCODED_HPP
#define CAMERA__ENCODED_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>
#include "mnt_surveillance_camera_node/codecs/codec_v1.hpp"
#include <chrono>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>
#include <opencv2/opencv.hpp>

using namespace std::chrono_literals;
using mnt_surveillance::camera_node::codec::CodecV1; // this is the codec class

namespace mnt_surveillance
{
    namespace camera_node
    {
        namespace camera
        {
            class CameraEncoded
            {
            public:
                explicit CameraEncoded(std::shared_ptr<rclcpp::Node> &nh,
                                const std::string &topic_name_);
                virtual ~CameraEncoded() = default;

                virtual bool open() = 0;
                virtual bool close() = 0;
                virtual cv::Mat capture() = 0;
                virtual cv::Mat process_image(cv::Mat);
                virtual void publish_capture();
                virtual void publish_encoded_data();
                virtual void publish_decoded_image();

            protected:
                void img_pub_callback();

                // ros variables
                std::shared_ptr<rclcpp::Node> nh_;
                rclcpp::QoS qos_ = rclcpp::QoS(rclcpp::KeepLast(10));
                rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr img_pub_;
                sensor_msgs::msg::Image::SharedPtr img_msg_;

                // encoded img publisher variables
                rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr encoded_data_pub_;
                std_msgs::msg::UInt8MultiArray::SharedPtr encoded_data_msg_;

                // decoded img publisher variables
                rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr  decoded_img_pub_;
                sensor_msgs::msg::Image::SharedPtr decoded_img_msg_;

                // openCV variables
                cv::Mat frame_;
                int frame_width_px_;
                int frame_height_px_;

                // codec variables
                std::shared_ptr<CodecV1> codec_;
            };

        }

    } // namespace camera
} // namespace mnt_surveillance

#endif // CAMERA__Encoded_HPP
