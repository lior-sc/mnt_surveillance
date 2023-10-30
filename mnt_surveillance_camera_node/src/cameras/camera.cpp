#include "mnt_surveillance_camera_node/cameras/camera.hpp"

using mnt_surveillance::camera_node::camera::Camera;

Camera::Camera(std::shared_ptr<rclcpp::Node> &nh,
               const std::string &topic_name = "")
    : nh_(nh),
      topic_name_(topic_name)
{
  RCLCPP_INFO(nh_->get_logger(), "Init Camera object");

  img_pub_ = nh_->create_publisher<sensor_msgs::msg::Image>(topic_name_, 10);
}

void Camera::publish()
{
  capture();
  img_pub_->publish(*msg_.get());
  // RCLCPP_INFO(nh_->get_logger(), "Image published");
}

cv::Mat Camera::process_image(cv::Mat img)
{
  // Crop image into a square shape
  int center_x = img.cols / 2;
  int center_y = img.rows / 2;
  int crop_size = std::min(img.rows, img.cols);

  int x = center_x - crop_size / 2;
  int y = center_y - crop_size / 2;

  cv::Rect rect(x, y, crop_size, crop_size);
  cv::Mat cropped_img = img(rect);

  // convert the image to grayscale and 16 bit depth
  cv::Mat gray_16bit_img;
  cv::cvtColor(cropped_img, gray_16bit_img, cv::COLOR_RGB2GRAY);
  // cropped_img.convertTo(grayscale_16bit_img, CV_16U, 1023.0 / 255.0);

  // resize the image to a 9 by 9 pixel matrix
  cv::Mat output_img;
  cv::Size output_img_size(9, 9);
  cv::resize(gray_16bit_img, output_img, output_img_size);

  return output_img;
}
