#include <rclcpp/rclcpp.hpp>
#include <rcutils/cmdline_parser.h>
#include "mnt_surveillance_master_node/recorder.hpp"

using mnt_surveillance::recorder_node::RecorderNode;

int main(int argc, char *argv[])
{
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);

    if (rcutils_cli_option_exist(argv, argv + argc, "-h"))
    {
        return 0;
    }

    rclcpp::init(argc, argv);

    rclcpp::executors::SingleThreadedExecutor executor;
    auto recorder_node = std::make_shared<RecorderNode>();

    executor.add_node(recorder_node);
    executor.spin();

    rclcpp::shutdown();

    return 0;
}