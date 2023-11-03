#ifndef MNT__RECORDER__HPP_
#define MNT__RECORDER__HPP_

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>
#include <fstream>
#include <filesystem>

namespace mnt_surveillance
{
    namespace recorder_node
    {
        class RecorderNode : public rclcpp::Node
        {
        public:
            explicit RecorderNode();
            ~RecorderNode();
        
        private:
            void start_recording(const std::string &topic_name);
            void stop_recording();
            void data_callback(const std_msgs::msg::UInt8MultiArray::SharedPtr msg);
            bool create_file_directory_path(const std::string &file_path);
            bool append_data_to_file(const std::vector<uint8_t> &data,
                                        const std::string &file_path);
            void declare_parameters();
            void get_parameters();

            // ros variables
            rclcpp::Subscription<std_msgs::msg::UInt8MultiArray>::SharedPtr data_subscriber_;
            std_msgs::msg::UInt8MultiArray::SharedPtr msg_;

            std::string topic_name_;
            std::string file_path_;
        };
    }
}
#endif  // MNT__RECORDER__HPP_