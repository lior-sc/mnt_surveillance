#ifndef CAMRAS__MP4__VIDEO_HPP_
#define CAMRAS__MP4__VIDEO_HPP_

#include "mnt_surveillance_camera_node/cameras/camera.hpp"

namespace mnt_surveillance
{
    namespace camera_node
    {
        namespace camera
        {
            class Mp4Video : public Camera
            {
            public:
                explicit Mp4Video(
                    std::shared_ptr<rclcpp::Node> &nh,
                    const std::string &topic_name);

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