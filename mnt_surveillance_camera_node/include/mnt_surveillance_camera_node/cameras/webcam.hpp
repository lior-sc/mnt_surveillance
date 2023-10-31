#ifndef CAMRAS__WEBCAM_HPP_
#define CAMRAS__WEBCAM_HPP_

#include "mnt_surveillance_camera_node/cameras/camera.hpp"

namespace mnt_surveillance
{
    namespace camera_node
    {
        namespace camera
        {
            class Webcam : public Camera
            {
            public:
                explicit Webcam(
                    std::shared_ptr<rclcpp::Node> &nh,
                    const std::string &topic_name = "/video/Image");

                bool capture() override;
                bool open() override;
                bool close() override;
                cv::Mat process_image(cv::Mat) override;

            private:
                rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr webcam_pub_;
                std::unique_ptr<cv::VideoCapture> cap;
            };

        }

    }
}

#endif