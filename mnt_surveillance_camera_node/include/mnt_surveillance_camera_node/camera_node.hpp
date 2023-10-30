#include <vector>
#include <rclcpp/rclcpp.hpp>
#include "mnt_surveillance_camera_node/cameras/camera.hpp"
#include "mnt_surveillance_camera_node/cameras/webcam.hpp"

namespace mnt_surveillance
{
    namespace camera_node
    {
        class CameraNode : public rclcpp::Node
        {
        public:
            CameraNode();

        private:
            void add_cameras();
            void run();
            void publish_video_streams();
            rclcpp::Node::SharedPtr node_handle_;
            rclcpp::TimerBase::SharedPtr publish_timer_;
            std::list<camera::Camera *> cameras_;

            // Other member functions here
        };

    } // namespace camera_node
} // namespace mnt_surveillance
