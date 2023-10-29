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
            CameraNode() : Node("camera_node")
            {
                // Constructor logic here
            }

        private:
            std::vector<Camera> cameras_;

            void add_cameras();

            // Other member functions here
        };

    } // namespace camera_node
} // namespace mnt_surveillance
