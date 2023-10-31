#include "mnt_surveillance_master_node/analyzers/analyzer.hpp"

using mnt_surveillance::master_node::analyzer::Analyzer;
using std::placeholders::_1;

Analyzer::Analyzer(std::shared_ptr<rclcpp::Node> &nh, const std::string &topic_name) : nh_(nh)
{
    // img_subscription_ = nh_->create_subscription<sensor_msgs::msg::Image>(
    //     topic_name, qos_, std::bind(&Analyzer::img_sub_callback, this, _1));
    create_img_subscriber(topic_name);
    create_alarm_service_client("/mnt_alarm");
}

void Analyzer::create_img_subscriber(const std::string &topic_name)
{
    img_subscription_ = nh_->create_subscription<sensor_msgs::msg::Image>(
        topic_name, qos_, std::bind(&Analyzer::img_sub_callback, this, _1));
}

void Analyzer::img_sub_callback(const sensor_msgs::msg::Image::SharedPtr msg)
{
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO16);
    }
    catch (cv_bridge::Exception &e)
    {
        RCLCPP_ERROR(nh_->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }

    // store frame in global variable
    frame_ = cv_ptr->image;
}

double Analyzer::get_saturated_pixels_ratio(cv::Mat frame, double threshold_value, double max_value)
{
    cv::Mat saturated_frame;
    cv::threshold(frame, saturated_frame, threshold_value, max_value, cv::THRESH_BINARY);

    int frame_pixel_num = static_cast<int>(frame.total());
    int saturated_frame_pixel_num = cv::countNonZero(saturated_frame);

    double saturated_pixels_ratio = static_cast<double>(saturated_frame_pixel_num) / static_cast<double>(frame_pixel_num); // ranges from 0 to 1

    return saturated_pixels_ratio;
}

double Analyzer::get_dark_pixels_ratio(cv::Mat frame, double threshold_value, double min_value)
{
    cv::Mat saturated_frame;
    cv::threshold(frame, saturated_frame, threshold_value, min_value, cv::THRESH_BINARY);

    int total_pixel_num = static_cast<int>(frame.total());
    int dark_pixel_num = total_pixel_num - cv::countNonZero(saturated_frame);

    double dark_pixels_ratio = static_cast<double>(dark_pixel_num) / static_cast<double>(total_pixel_num); // ranges from 0 to 1

    return dark_pixels_ratio;
}

void Analyzer::check_alarm_conditions()
{

    /**
     * if saturated pixels above 20% that means someone is blinding the camera
     */
    double saturation_thresh = 0.2;
    double saturation_ratio = get_saturated_pixels_ratio(frame_, 1022, 1023);

    if (saturation_ratio > saturation_thresh)
    {
        alarm_request_->a = 2;
        alarm_request_->b = 3;
        alarm_client_->async_send_request(alarm_request_);
    }

    double darkness_thresh = 0.81;
    double darkness_ratio = get_dark_pixels_ratio(frame_, 100, 1023);

    /**
     * if more than 80% of the camera is dark than someone has covered the camera
     */
    if (darkness_ratio > darkness_thresh)
    {
        alarm_request_->a = 4;
        alarm_request_->b = 5;
        alarm_client_->async_send_request(alarm_request_);
    }
}

void Analyzer::create_alarm_service_client(const std::string &service_name)
{
    alarm_client_ = nh_->create_client<example_interfaces::srv::AddTwoInts>(service_name);
    alarm_request_ = std::make_shared<example_interfaces::srv::AddTwoInts::Request>();
}