#include <rclcpp/rclcpp.hpp>
#include "mnt_surveillance_camera_node/cameras/camera.hpp"
#include "mnt_surveillance_camera_node/cameras/camera_encoded.hpp"
#include "mnt_surveillance_camera_node/cameras/webcam.hpp"
#include "mnt_surveillance_camera_node/cameras/webcam_encoded.hpp"  
#include "mnt_surveillance_camera_node/cameras/random_noise.hpp"
#include "mnt_surveillance_camera_node/cameras/fixed_frame_encoded.hpp"

namespace mnt_surveillance
{
    namespace camera_node
    {
        class CameraNode : public rclcpp::Node
        {
        public:
            CameraNode(const std::string &camera_node_name);

        private:
            void declare_parameters();
            void add_cameras();
            void run();
            void publish_video_streams();
            rclcpp::Node::SharedPtr node_handle_;
            rclcpp::TimerBase::SharedPtr publish_timer_;
            std::list<camera::Camera *> cameras_;
            std::list<camera::CameraEncoded *> encoded_cameras_;

            // Other member functions here
        };

    } // namespace camera_node
} // namespace mnt_surveillance
