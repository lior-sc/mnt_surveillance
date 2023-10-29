#include <rclcpp/rclcpp.hpp>
#include <rcutils/cmdline_parser.h>
#include "mnt_surveillance_camera_node/camera_node.hpp"

using mnt_surveillance::camera_node::CameraNode;

int main(int argc, char *argv[])
{
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);

    if (rcutils_cli_option_exist(argv, argv + argc, "-h"))
    {
        return 0;
    }

    rclcpp::init(argc, argv);

    auto camera1_node = std::make_shared<CameraNode>();

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(camera1_node);
    executor.spin();

    rclcpp::shutdown();

    return 0;
}