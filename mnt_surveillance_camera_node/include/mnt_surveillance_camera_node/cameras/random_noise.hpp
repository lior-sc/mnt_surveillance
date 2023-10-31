#ifndef CAMRAS__RANDOM__NOISE_HPP_
#define CAMRAS__RANDOM__NOISE_HPP_

#include "mnt_surveillance_camera_node/cameras/camera.hpp"

namespace mnt_surveillance
{
    namespace camera_node
    {
        namespace camera
        {
            class RandomNoise : public Camera
            {
            public:
                explicit RandomNoise(
                    std::shared_ptr<rclcpp::Node> &nh,
                    const std::string &topic_name = "/video/Image");

                bool capture() override;
                bool open() override;
                bool close() override;

            private:
                rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr webcam_pub_;
            };

        } // namespace camera

    } // namespace camera_node
} // mnt_surveillance

#endif // CAMRAS__RANDOM__NOISE_HPP_