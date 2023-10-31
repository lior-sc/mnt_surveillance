#include <rclcpp/rclcpp.hpp>
#include <rcutils/cmdline_parser.h>
#include "mnt_surveillance_master_node/master_node.hpp";

using mnt_surveillance::master_node::MasterNode;

int main(int argc, char *argv[])
{
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);

    if (rcutils_cli_option_exist(argv, argv + argc, "-h"))
    {
        return 0;
    }

    rclcpp::init(argc, argv);

    rclcpp::executors::SingleThreadedExecutor executor;
    auto master_node = std::make_shared<MasterNode>();

    executor.add_node(master_node);
    executor.spin();

    rclcpp::shutdown();

    return 0;
}