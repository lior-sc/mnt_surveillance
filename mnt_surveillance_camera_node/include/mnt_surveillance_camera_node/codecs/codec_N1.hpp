#ifndef IMAGE__CODEC__N1_HPP
#define IMAGE__CODEC__N1_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/u_int16_multi_array.hpp>

namespace mnt_surveillance
{
    namespace camera_node
    {
        namespace codec
        {
            class Codec_N1
            {
            public:
                explicit Codec_N1();
                ~Codec_N1() = default;

                std::list<uint16_t> decode_data(std::list<uint8_t> encoded_data);
                std::list<uint8_t> encode_data(std::list<uint16_t> decode_data);
            };
        } // namespace codec
    }
}

#endif // IMAGE__CODEC__N1_HPP