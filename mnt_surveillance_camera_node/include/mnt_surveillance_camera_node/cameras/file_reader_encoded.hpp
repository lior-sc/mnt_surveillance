#ifndef FILE_READER__ENCODED_HPP_
#define FILE_READER__ENCODED_HPP_

#include "mnt_surveillance_camera_node/cameras/camera_encoded.hpp"
#include <fstream>

namespace mnt_surveillance
{
    namespace camera_node
    {
        namespace camera
        {
            class FileReaderEncoded : public CameraEncoded
            {
            public:
                explicit FileReaderEncoded(
                    std::shared_ptr<rclcpp::Node> &nh,
                    const std::string &topic_name = "/video/raw_data");

                cv::Mat capture() override;
                bool open() override;
                bool close() override;
                // cv::Mat process_image(cv::Mat) override;

            private:
                rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr webcam_pub_;
                std::unique_ptr<cv::VideoCapture> cap;
            };

        }

    }
}

#endif