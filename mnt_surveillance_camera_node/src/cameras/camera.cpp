#include "mnt_surveillance_camera_node/cameras/camera.hpp"
#include "mnt_surveillance_camera_node/codecs/codec_v1.hpp"

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
