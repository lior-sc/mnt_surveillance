#include "mnt_surveillance_master_node/analyzers/analyzer.hpp"

using mnt_surveillance::master_node::analyzer::Analyzer;
using std::placeholders::_1;

void print_pixel_values(cv::Mat frame);
double get_oversaturated_pixels(cv::Mat frame, int thresold);

Analyzer::Analyzer(std::shared_ptr<rclcpp::Node> &nh,
                   const std::string &img_topic_name,
                   const std::string &alarm_service_name)
    : nh_(nh)
{
    RCLCPP_INFO(nh_->get_logger(), "Init analyzer object");
    
    get_parameters();
    create_img_subscriber(img_topic_name);
    create_alarm_service_client(alarm_service_name);
}

void Analyzer::get_parameters()
{
    // get and store paremeters from yaml file
    nh_->get_parameter_or("alarm_over_saturation_flag",alarm_over_saturation_flag_, true);
    nh_->get_parameter_or("alarm_over_saturation_threshold",saturation_thresh_, 30000);
    nh_->get_parameter_or("alarm_over_saturation_ratio_threshold",saturation_ratio_thresh_, 0.20);
    
    nh_->get_parameter_or("alarm_under_saturation_flag",alarm_under_saturation_flag_, true);
    nh_->get_parameter_or("alarm_under_saturation_threshold",undersaturation_thresh_, 30000);
    nh_->get_parameter_or("alarm_under_saturation_ratio_threshold",undersaturation_ratio_thresh_, 0.81);

    nh_->get_parameter_or("analyzer_print_results_flag",print_results_flag_, false);
}

void Analyzer::create_img_subscriber(const std::string &topic_name)
{
    img_subscription_ = nh_->create_subscription<sensor_msgs::msg::Image>(
        topic_name, qos_, std::bind(&Analyzer::img_sub_callback, this, _1));
}

void Analyzer::img_sub_callback(const sensor_msgs::msg::Image::SharedPtr msg)
{
    // get image
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

    // check if alarm has to be set
    check_alarm_conditions();
}

double Analyzer::get_saturated_pixels_ratio(cv::Mat frame, double threshold_value, double max_pixel_value)
{
    cv::Mat saturated_frame;
    cv::threshold(frame, saturated_frame, threshold_value, max_pixel_value, cv::THRESH_BINARY);  

    int frame_pixel_num = static_cast<int>(frame.total());
    int saturated_frame_pixel_num = cv::countNonZero(saturated_frame);

    double saturated_pixels_ratio = static_cast<double>(saturated_frame_pixel_num) / static_cast<double>(frame_pixel_num); // ranges from 0 to 1

    return saturated_pixels_ratio;
}

double Analyzer::get_dark_pixels_ratio(cv::Mat frame, double threshold_value, double max_pixel_value)
{
    cv::Mat saturated_frame;
    cv::threshold(frame, saturated_frame, threshold_value, max_pixel_value, cv::THRESH_BINARY);

    int total_pixel_num = static_cast<int>(frame.total());
    int dark_pixel_num = total_pixel_num - cv::countNonZero(saturated_frame);

    double dark_pixels_ratio = static_cast<double>(dark_pixel_num) / static_cast<double>(total_pixel_num); // ranges from 0 to 1

    return dark_pixels_ratio;
}

void Analyzer::check_alarm_conditions()
{

    if(alarm_over_saturation_flag_ == true)
    {
        // double saturation_ratio = get_saturated_pixels_ratio(frame_, saturation_thresh_, 65535);
        double saturation_ratio = get_saturated_pixels_ratio(frame_, 
                                                             static_cast<double>(saturation_thresh_), 
                                                             65535);

        if(print_results_flag_ == true)
        {
            // print results
            RCLCPP_INFO(nh_->get_logger(), "Analyzer over saturation ratio: %f", saturation_ratio);
        }

        if (saturation_ratio > saturation_ratio_thresh_)
        {
            alarm_client_->async_send_request(alarm_request_);
        }
    }

    if(alarm_under_saturation_flag_ == true)
    {
        double undersaturation_ratio = get_dark_pixels_ratio(frame_,
                                                             static_cast<double>(undersaturation_thresh_), 
                                                             65535);

        if(print_results_flag_ == true)
        {
            // print results
            RCLCPP_INFO(nh_->get_logger(), "Analyzer undersaturation ratio: %f", undersaturation_ratio);
        }

        if (undersaturation_ratio > undersaturation_ratio_thresh_)
        {
            alarm_client_->async_send_request(alarm_request_);
        }
    }

    return;
}

void Analyzer::create_alarm_service_client(const std::string &service_name)
{
    alarm_client_ = nh_->create_client<std_srvs::srv::Trigger>(service_name);
    alarm_request_ = std::make_shared<std_srvs::srv::Trigger::Request>();
}




// void print_pixel_values(cv::Mat frame)
// {
//     for (int i = 0; i < frame.rows; i++) {
//         for (int j = 0; j < frame.cols; j++) {
//             std::cout << frame.at<uint16_t>(i, j) << " ";
//         }
//         std::cout << std::endl;
//     }
// }

// double get_oversaturated_pixels(cv::Mat frame, int thresold)
// {
//     int total_pixels = static_cast<int>(frame.total());
//     int oversaturated_pixels = 0;
//     for (int i = 0; i < frame.rows; i++) {
//         for (int j = 0; j < frame.cols; j++) {
//             if (frame.at<uint16_t>(i, j) > static_cast<uint16_t>(thresold))
//             {
//                 oversaturated_pixels++;   
//             }
//         }
//     }
    
//     double oversaturated_pixels_ratio = static_cast<double>(oversaturated_pixels) / static_cast<double>(total_pixels); // ranges from 0 to 1

//     std::cout << "Oversaturated pixels: " << oversaturated_pixels << std::endl;
//     return oversaturated_pixels_ratio;
// }