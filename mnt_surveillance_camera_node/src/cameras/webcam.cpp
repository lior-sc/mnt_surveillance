#include "mnt_surveillance_camera_node/cameras/webcam.hpp"
#include "mnt_surveillance_camera_node/misc/stopwatch.hpp"

using mnt_surveillance::camera_noda::StopWatch;
using mnt_surveillance::camera_node::camera::Webcam;

Webcam::Webcam(
    std::shared_ptr<rclcpp::Node> &nh,
    const std::string &topic_name)
    : Camera(nh, topic_name)
{
    this->open();
    // hello
}

bool Webcam::capture()
{
    cv::Mat frame;
    cap->read(frame);

    cv::Mat proc_frame = this->process_image(frame);

    msg_ = cv_bridge::CvImage(std_msgs::msg::Header(), "mono16", proc_frame).toImageMsg();
    return true;
}

bool Webcam::open()
{
    RCLCPP_INFO(nh_->get_logger(), "opening camera");
    cap = std::make_unique<cv::VideoCapture>(0);

    std::this_thread::sleep_for(std::chrono::seconds(1)); // add this line

    if (!cap->isOpened())
    {
        RCLCPP_ERROR(nh_->get_logger(), "failed to open teh webcam. change camera port number");
        this->close();
        return false;
    }

    RCLCPP_INFO(nh_->get_logger(), "camera works!");
    RCLCPP_INFO(nh_->get_logger(), "Succeeded to create webcam publisher");

    return true;
}

bool Webcam::close()
{
    // do nothing
    cap->release();
    return true;
}

cv::Mat Webcam::process_image(cv::Mat img)
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
