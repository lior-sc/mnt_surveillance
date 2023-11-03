#ifndef IMAGE__CODEC__N1_HPP
#define IMAGE__CODEC__N1_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>
#include <opencv2/opencv.hpp>

namespace mnt_surveillance
{
    namespace master_node
    {
        namespace codec
        {
            class CodecV1
            {
            public:
                explicit CodecV1(int frame_width_px, int frame_height_px);
                ~CodecV1() = default;

                int get_frame_width_px();
                int get_frame_height_px();
                void set_frame_size_px(int width, int height);
                std::vector<uint16_t> decode_data(std::vector<uint8_t> encoded_data);
                std::vector<uint8_t> encode_data(std::vector<uint16_t> decode_data);
                std_msgs::msg::UInt8MultiArray uint8_vector_to_msg(std::vector<uint8_t> input_data);
                std::vector<uint8_t> msg_to_uint8_vector(std_msgs::msg::UInt8MultiArray input_msg);
                sensor_msgs::msg::Image uint16_vector_to_ros_image(std::vector<uint16_t> input_data);
                sensor_msgs::msg::Image decode_to_ros_image(std::vector<uint8_t> input_data);
                std::vector<uint16_t> get_pixel_vector(cv::Mat input_frame);

                private:
                int frame_width_px_;
                int frame_height_px_;


            };
        } // namespace codec
    }
}

#endif // IMAGE__CODEC__N1_HPP