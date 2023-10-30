#include <vector>
#include <rclcpp/rclcpp.hpp>
#include "mnt_surveillance_camera_node/cameras/camera.hpp"

namespace mnt_surveillance
{
    namespace camera_node
    {
        using mnt_surveillance::camera::Camera;

        class CameraNode : public rclcpp::Node
        {
        public:
            CameraNode();

        private:
            void add_cameras();
            void run();
            void publish_video_streams();

            std::vector<Camera> cameras_;
            rclcpp::Node::SharedPtr node_handle_;
            rclcpp::TimerBase::SharedPtr publish_timer_;

            // Other member functions here
        };

    } // namespace camera_node
} // namespace mnt_surveillance
