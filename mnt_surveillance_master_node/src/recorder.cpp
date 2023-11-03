#include "mnt_surveillance_master_node/recorder.hpp"

using mnt_surveillance::recorder_node::RecorderNode;
using std::placeholders::_1;

RecorderNode::RecorderNode() : Node("recorder_node") 
{
    RCLCPP_INFO(this->get_logger(), "Init recorder object");
    
    declare_parameters();
    get_parameters();

    RCLCPP_INFO(this->get_logger(), "Recording topic: %s to path: %s", topic_name_.c_str(), file_path_.c_str());

    bool success = create_file_directory_path(file_path_);

    if(!success)
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to create file directory path");
        RCLCPP_ERROR(this->get_logger(), "This node will remain stagnant. consider re-launching the node");
        return;
    }

    RCLCPP_INFO(this->get_logger(), "file directory path created successfully");
    
    start_recording(topic_name_);

    return;
}

RecorderNode::~RecorderNode()
{
    RCLCPP_INFO(this->get_logger(), "Destroy recorder object");
    stop_recording();
}

void RecorderNode::start_recording(const std::string &topic_name)
{
    RCLCPP_INFO(this->get_logger(), "Start recording");
    data_subscriber_ = this->create_subscription<std_msgs::msg::UInt8MultiArray>(
        topic_name, 10, std::bind(&RecorderNode::data_callback, this, _1));
}

void RecorderNode::stop_recording()
{
    RCLCPP_INFO(this->get_logger(), "Stop recording");
    data_subscriber_.reset();
}

void RecorderNode::data_callback(const std_msgs::msg::UInt8MultiArray::SharedPtr msg)
{
    RCLCPP_INFO(this->get_logger(), "Data callback");

    std::vector<uint8_t> data = msg->data;
    append_data_to_file(data, "/home/lior/Desktop/test.txt");
}

bool RecorderNode::create_file_directory_path(const std::string &file_path)
{
    RCLCPP_INFO(this->get_logger(), "Create file directory path");

    std::filesystem::path directory_path = std::filesystem::path(file_path).parent_path();

    bool success;

    if (!std::filesystem::exists(directory_path))
    {
        success = std::filesystem::create_directories(directory_path);

        if(!success)
            RCLCPP_ERROR(this->get_logger(), "Failed to create directory path");
        else
            RCLCPP_INFO(this->get_logger(), "Created directory path");
    }

    return success;
}

bool RecorderNode::append_data_to_file(const std::vector<uint8_t> &data,
                                       const std::string &file_path)
{
    std::ofstream file(file_path, std::ios::binary | std::ios::app);
    if (!file.is_open())
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to open file");
        return false;
    }

    file.write(reinterpret_cast<const char *>(data.data()), data.size());

    if (file.good())
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to write to file");
        return false;
    }

    file.close();

    return true;
}

void RecorderNode::declare_parameters()
{
    // declare all parameters present in yaml file
    this->declare_parameter<std::string>("topic_name", "/video/raw_data");
    this->declare_parameter<std::string>("file_path", "l/home/lior/Desktop/mnt_surveillance_data.bin");
}

void RecorderNode::get_parameters()
{
    // get and store paremeters from yaml file
    this->get_parameter_or("topic_name", 
                            topic_name_, 
                            std::string("/video/raw_data"));
    
    this->get_parameter_or("file_path", 
                            file_path_, 
                            std::string("/home/lior/Desktop/mnt_surveillance_data.bin"));
}
                                       