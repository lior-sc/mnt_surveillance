#ifndef IMG__DECODER__PUBLISHER_HPP_
#define IMG__DECODER__PUBLISHER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>
#include "mnt_surveillance_master_node/codecs/codec_v1.hpp"

namespace mnt_surveillance
{
    namespace master_node
    {
        namespace img_decoder
        {
            class ImgDecoder
            {
            public:
                explicit ImgDecoder(std::shared_ptr<rclcpp::Node> &nh,
                                    std::string encoded_video_topic,
                                    std::string decoded_video_topic);
                virtual ~ImgDecoder() = default;

                private:
                void create_img_publisher(const std::string &topic_name);
                void create_raw_data_subscriber(const std::string &topic_name);
                void raw_data_callback(const std_msgs::msg::UInt8MultiArray::SharedPtr msg);

                // ros variables
                std::shared_ptr<rclcpp::Node> nh_;
                rclcpp::QoS qos_ = rclcpp::QoS(rclcpp::KeepLast(10));

                // img publisher
                rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr img_publisher_;
                sensor_msgs::msg::Image::SharedPtr msg_;

                // raw data subscriber
                rclcpp::Subscription<std_msgs::msg::UInt8MultiArray>::SharedPtr raw_data_subscriber_;
                std_msgs::msg::UInt8MultiArray::SharedPtr raw_data_msg_;

                std::shared_ptr<codec::CodecV1> codec_;
                int frame_width_px_;
                int frame_height_px_;
            };

        }

    } // namespace camera

} // namespace mnt_surveillance


#endif  // IMG__DECODER__PUBLISHER_HPP_