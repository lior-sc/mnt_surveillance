#include "mnt_surveillance_camera_node/codecs/codec_v1.hpp"

using mnt_surveillance::camera_node::codec::CodecV1;

CodecV1::CodecV1(int frame_width_px, int frame_height_px)
    : frame_width_px_(frame_width_px),
      frame_height_px_(frame_height_px)
{
    // do nothing
}

int CodecV1::get_frame_width_px()
{
    return frame_width_px_;
}

int CodecV1::get_frame_height_px()
{
    return frame_height_px_;
}

void CodecV1::set_frame_size_px(int width, int height)
{
    frame_width_px_ = width;
    frame_height_px_ = height;
    return;
}

std::vector<uint16_t> CodecV1::decode_data(std::vector<uint8_t> encoded_data)
{
    /**
     * I know this is not the most efficient way but lets go with it for now
    */
    std::vector<uint8_t>::iterator it = encoded_data.begin();
    std::vector<uint16_t> decoded_data;

    uint16_t decoded_pixel = 0;
    uint8_t encoded_byte = 0;
    int bit_counter = 0;

    while(it != encoded_data.end())
    {
        encoded_byte = *it;
        for (int i=2;i<=8;i+=2)
        {
            decoded_pixel = decoded_pixel | ((encoded_byte >> (8-i)) & 0x03);
            bit_counter += 2;
            if(bit_counter >= 10)
            {
                // move data to msb;
                decoded_data.push_back(decoded_pixel);
                decoded_pixel = 0;
                bit_counter = 0;
            }
            else
            {
                decoded_pixel = decoded_pixel << 2;
            }
        }
        it++;
    }
    return decoded_data;
}

std::vector<uint8_t> CodecV1::encode_data(std::vector<uint16_t> raw_data)
{
    /**
     * I know this is not the most efficient way but lets go with it for now
    */
    std::vector<uint16_t>::iterator it = raw_data.begin();
    std::vector<uint8_t> encoded_data;
    uint16_t raw_pixel = 0;
    uint8_t encoded_byte = 0;
    int bit_counter = 0;

    while(it != raw_data.end())
    {
        raw_pixel = *it;

        for (int i=2;i<=10;i+=2)
        {
            encoded_byte = encoded_byte | ((raw_pixel >> (10-i)) & 0x03);
            bit_counter += 2;
            if(bit_counter >= 8)
            {
                encoded_data.push_back(encoded_byte);
                encoded_byte = 0;
                bit_counter = 0;
            }
            else
            {
                encoded_byte = encoded_byte << 2;
            }
        }

        it++;
    }
    return encoded_data;
}

std_msgs::msg::UInt8MultiArray CodecV1::uint8_vector_to_msg(std::vector<uint8_t> input_data)
{
    std_msgs::msg::UInt8MultiArray msg;
    for(auto it = input_data.begin(); it != input_data.end(); it++)
    {
        msg.data.push_back(*it);
    }
    return msg;
}

std::vector<uint8_t> CodecV1::msg_to_uint8_vector(std_msgs::msg::UInt8MultiArray input_msg)
{
    std::vector<uint8_t> output_vector;
    for(auto it = input_msg.data.begin(); it != input_msg.data.end(); it++)
    {
        output_vector.push_back(*it);
    }
    return output_vector;
}

sensor_msgs::msg::Image CodecV1::uint16_vector_to_ros_image(std::vector<uint16_t> input_data)
{
    sensor_msgs::msg::Image msg;
    msg.header.stamp = rclcpp::Time();
    msg.height = frame_height_px_;
    msg.width = frame_width_px_;
    msg.encoding = "mono16";
    msg.is_bigendian = false;
    msg.step = frame_width_px_ * sizeof(uint16_t);
    msg.data.resize(msg.step * msg.height);
    
    int i = 0;
    for(auto it = input_data.begin(); it != input_data.end(); it++)
    {
        uint8_t lowByte = static_cast<uint8_t>(*it & 0x00FF);
        uint8_t highByte = static_cast<uint8_t>((*it >> 8) & 0x00FF);
        msg.data[i] = highByte;
        msg.data[i+1] = lowByte;
        i+=2;
    }
    
    return msg;
}

sensor_msgs::msg::Image CodecV1::decode_to_ros_image(std::vector<uint8_t> input_data)
{
    std::vector<uint16_t> decoded_data = decode_data(input_data);
    sensor_msgs::msg::Image ros_image = uint16_vector_to_ros_image(decoded_data);
    
    return ros_image;
}

std::vector<uint16_t> CodecV1::get_pixel_vector(cv::Mat input_frame)
{
    std::vector<uint16_t> pixel_vector;
    
    for(int i=0; i<input_frame.rows; i++)
    {
        for(int j=0;j<input_frame.cols;j++)
        {
            pixel_vector.push_back(input_frame.at<uint16_t>(i,j));
        }
    }
    return pixel_vector;
}
