#include <omp.h>
#include <mutex>
#include <thread>
#include <fstream>
#include <csignal>
#include <chrono>
#include <unistd.h>

#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <livox_ros_driver2/msg/custom_msg.hpp>

#include "preprocess.h"
#include <fastlio.h>

class NodeCloudProcess : public rclcpp::Node
{
public:

    // 结构体初始化
    NodeCloudProcess(const std::string & name);

    // 析构函数
    ~NodeCloudProcess();

private:

    void declare_and_get_parameter();

    void standard_pcl_cbk(const sensor_msgs::msg::PointCloud2::UniquePtr msg);

    double timediff_lidar_wrt_imu = 0.0;
    bool   timediff_set_flg = false;
    void livox_pcl_cbk(const livox_ros_driver2::msg::CustomMsg::UniquePtr msg);

    void imu_cbk(const sensor_msgs::msg::Imu::UniquePtr msg_in);

    string lid_topic, imu_topic;
    bool time_sync_en = false;
    double time_diff_lidar_to_imu = 0.0;
    bool is_first_lidar = true;
    
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_;                     // 创建一个订阅器，用于订阅IMU数据。
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_pcl_pc_;          // 创建一个订阅器，用于订阅原始点云数据。
    rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr sub_pcl_livox_;   // 创建一个订阅器，用于订阅 Livox 点云数据。

    mutex mtx_buffer;
    condition_variable sig_buffer;

    int publish_count=0, scan_count=0;

    shared_ptr<Preprocess> preprocess=std::make_shared<Preprocess>();
    
};
