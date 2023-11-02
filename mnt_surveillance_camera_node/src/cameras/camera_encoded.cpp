#include "mnt_surveillance_camera_node/cameras/camera_encoded.hpp"
#include "mnt_surveillance_camera_node/codecs/codec_v1.hpp"

using mnt_surveillance::camera_node::camera::CameraEncoded;

CameraEncoded::CameraEncoded(std::shared_ptr<rclcpp::Node> &nh,
               const std::string &topic_name)
    : nh_(nh)
{
  RCLCPP_INFO(nh_->get_logger(), "Init CameraEncoded object");

  // get parameters

  nh_->get_parameter("frame_width_px", frame_width_px_); // fixed line
  nh_->get_parameter("frame_height_px", frame_height_px_); // fixed line

  codec_ = std::make_shared<CodecV1>(frame_width_px_, frame_height_px_);

  // create publishers
  img_pub_ = nh_->create_publisher<sensor_msgs::msg::Image>(topic_name, qos_);
  encoded_data_pub_ = nh_->create_publisher<std_msgs::msg::UInt8MultiArray>(topic_name + "/encoded", qos_);
  decoded_img_pub_ = nh_->create_publisher<sensor_msgs::msg::Image>(topic_name + "/decoded", qos_);
  
}


void CameraEncoded::publish_capture()
{
  frame_ = capture();
  cv::Mat processed_frame = process_image(frame_);
  img_msg_ = cv_bridge::CvImage(std_msgs::msg::Header(), "mono16", processed_frame).toImageMsg();
  img_pub_->publish(*img_msg_.get());
}

void CameraEncoded::publish_decoded_image()
{
  frame_ = capture();
  cv::Mat processed_frame = process_image(frame_);
  std::vector<uint16_t> frame_data = codec_->get_pixel_vector(processed_frame);
  std::vector<uint8_t> encoded_data = codec_->encode_data(frame_data);
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

  decoded_img_pub_->publish(decoded_img_msg);

  return;
}

void CameraEncoded::publish_encoded_data()
{
  frame_ = capture();
  cv::Mat processed_frame = process_image(frame_);

  codec_->set_frame_size_px(processed_frame.cols / sizeof(uint16_t), processed_frame.rows);
  std::vector<uint8_t> encoded_data = codec_->encode_data(codec_->get_pixel_vector(processed_frame));

  // RCLCPP_INFO(nh_->get_logger(), "encoded data first bytes: %d, %d, %d, %d", encoded_data[0], encoded_data[1], encoded_data[2], encoded_data[3]);

  std_msgs::msg::UInt8MultiArray encoded_data_msg;
  encoded_data_msg.data = encoded_data;

  encoded_data_pub_->publish(encoded_data_msg);
  return;
}


cv::Mat CameraEncoded::process_image(cv::Mat img)
{
    // Crop image into a square shape
    int center_x = img.cols / 2;
    int center_y = img.rows / 2;
    int crop_size = std::min(img.rows, img.cols);

    int x = center_x - crop_size / 2;
    int y = center_y - crop_size / 2;

    cv::Rect rect(x, y, crop_size, crop_size);
    cv::Mat cropped_img = img(rect);

    // resize image
    cv::Mat resized_img;
    cv::Size resized_img_size(frame_height_px_, frame_width_px_);
    cv::resize(cropped_img, resized_img, resized_img_size);

    // convert the image to grayscale and 16 bit depth
    cv::Mat gray_img;
    cv::Mat gray_16bit_img;

    // convert to grayscale if the image is not grayscale
    if(img.channels() != 1){

    }

    if(img.type() != CV_16U)
    {
      // resized_img.convertTo(resized_img,CV_16U,1023.0/255.0);
    }

    cv::cvtColor(resized_img, resized_img, cv::COLOR_BGR2GRAY);
    resized_img.convertTo(resized_img,CV_16U,1023.0/255.0);

    return resized_img;
}
