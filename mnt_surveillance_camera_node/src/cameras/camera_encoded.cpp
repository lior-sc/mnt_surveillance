#include "mnt_surveillance_camera_node/cameras/camera_encoded.hpp"
#include "mnt_surveillance_camera_node/codecs/codec_v1.hpp"

using mnt_surveillance::camera_node::camera::CameraEncoded;

CameraEncoded::CameraEncoded(std::shared_ptr<rclcpp::Node> &nh,
               const std::string &topic_name)
    : nh_(nh)
{
  RCLCPP_INFO(nh_->get_logger(), "Init CameraEncoded object");

  // declare parameters
  nh_->declare_parameter<int>("frame_width_px");
  nh_->declare_parameter<int>("frame_height_px");

  // get parameters
  int frame_width_px;
  int frame_height_px;
  nh_->get_parameter("frame_width_px", frame_width_px); // fixed line
  nh_->get_parameter("frame_height_px", frame_height_px); // fixed line

  codec_ = std::make_shared<CodecV1>(frame_width_px, frame_height_px);

  // create publishers
  img_pub_ = nh_->create_publisher<sensor_msgs::msg::Image>(topic_name, qos_);
  encoded_data_pub_ = nh_->create_publisher<std_msgs::msg::UInt8MultiArray>(topic_name + "/encoded", qos_);
  decoded_img_pub_ = nh_->create_publisher<sensor_msgs::msg::Image>(topic_name + "/decoded", qos_);
  
}


void CameraEncoded::publish_capture()
{
  frame_ = capture();
  auto img_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "mono16", frame_).toImageMsg();
  img_pub_->publish(*img_msg.get());
}

void CameraEncoded::publish_decoded_image()
{
  frame_ = capture();
  cv::Mat processed_frame = process_image(frame_);
  std::vector<uint16_t> frame_data = codec_->get_pixel_vector(processed_frame);
  std::vector<uint8_t> encoded_data = codec_->encode_data(frame_data);
  
  std_msgs::msg::UInt8MultiArray encoded_data_msg_ = codec_->uint8_vector_to_msg(encoded_data);
  sensor_msgs::msg::Image decoded_img_msg = codec_->decode_to_ros_image(encoded_data_msg_.data);
  decoded_img_pub_->publish(decoded_img_msg);
}

void CameraEncoded::publish_encoded_data()
{
  frame_ = capture();
  cv::Mat processed_frame = process_image(frame_);

  codec_->set_frame_size_px(processed_frame.cols / sizeof(uint16_t), processed_frame.rows);
  std::vector<uint8_t> encoded_data = codec_->encode_data(codec_->get_pixel_vector(processed_frame));
  
  std_msgs::msg::UInt8MultiArray encoded_data_msg_ = codec_->uint8_vector_to_msg(encoded_data);
  encoded_data_pub_->publish(encoded_data_msg_);
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
    cv::Size resized_img_size(100, 100);
    cv::resize(cropped_img, resized_img, resized_img_size);

    // convert the image to grayscale and 16 bit depth
    cv::Mat gray_16bit_img;
    cv::Mat gray_img;
    cv::cvtColor(resized_img, gray_img, cv::COLOR_BGR2GRAY);
    gray_img.convertTo(gray_16bit_img, CV_16U, 1023.0 / 255.0);

    return gray_16bit_img;
}
