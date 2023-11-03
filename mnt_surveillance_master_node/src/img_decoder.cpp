#include "mnt_surveillance_master_node/img_decoder.hpp"

using mnt_surveillance::master_node::img_decoder::ImgDecoder;
using mnt_surveillance::master_node::codec::CodecV1;
using std::placeholders::_1;


ImgDecoder::ImgDecoder( std::shared_ptr<rclcpp::Node> &nh,
                        std::string encoded_video_topic,
                        std::string decoded_video_topic) :
    nh_(nh)
{
    std::string encoded_video_topic;
    std::string decoded_video_topic;
    
    //global parameters
    nh_->get_parameter_or("frame_width_px", 
                        frame_width_px_, 
                        9);
    nh_->get_parameter_or("frame_height_px", 
                        frame_height_px_, 
                        9);    
    std::make_shared<CodecV1>(frame_width_px_, frame_height_px_);
    
    create_img_publisher(decoded_video_topic);
    create_raw_data_subscriber(encoded_video_topic);
    
    return;
}

void ImgDecoder::create_raw_data_subscriber(const std::string &topic_name)
{
    raw_data_subscriber_ = nh_->create_subscription<std_msgs::msg::UInt8MultiArray>(
        topic_name, qos_, std::bind(&ImgDecoder::raw_data_callback, this, _1));
}

void ImgDecoder::raw_data_callback(const std_msgs::msg::UInt8MultiArray::SharedPtr msg)
{
    std::vector<uint8_t> encoded_data = msg->data;
    std::vector<uint16_t> decoded_data = codec_->decode_data(encoded_data);

    sensor_msgs::msg::Image decoded_img_msg;
    decoded_img_msg.header.stamp = nh_->now();
    decoded_img_msg.height = frame_height_px_;
    decoded_img_msg.width = frame_width_px_; 
    decoded_img_msg.encoding = "mono16";
    decoded_img_msg.is_bigendian = false;
    decoded_img_msg.step = frame_width_px_ * sizeof(uint16_t);
    decoded_img_msg.data.resize(decoded_img_msg.step * decoded_img_msg.height);

    for(size_t i=0; i<decoded_data.size(); i++)
    {
        uint8_t lowByte = static_cast<uint8_t>(decoded_data[i] & 0x00FF);
        uint8_t highByte = static_cast<uint8_t>((decoded_data[i] >> 8) & 0x00FF);
        decoded_img_msg.data[i*2] = highByte;
        decoded_img_msg.data[i*2+1] = lowByte;
    }

    // sensor_msgs::msg::Image decoded_img_msg = codec_->decode_to_ros_image(encoded_data_msg_.data);

    img_publisher_->publish(decoded_img_msg);

}

void ImgDecoder::create_img_publisher(const std::string &topic_name)
{
    img_publisher_ = nh_->create_publisher<sensor_msgs::msg::Image>(topic_name, qos_);
}

