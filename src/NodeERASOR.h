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

#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>

#include <sensor_msgs/msg/point_cloud2.hpp>

#include <fastlio.h>

#include "ERASOR/ERASOR.h"


#define ANSI_RED "\033[1m\x1b[31m"
#define ANSI_GREEN "\033[1m\x1b[32m"
#define ANSI_YELLOW "\033[1m\x1b[33m"
#define ANSI_BLUE "\033[1m\x1b[34m"
#define ANSI_MAGENTA "\033[1m\x1b[35m"
#define ANSI_CYAN "\033[1m\x1b[36m"
#define ANSI_RESET "\x1b[0m"
#define ANSI_BOLD "\033[1m"

#define NUM_PTS_LARGE_ENOUGH 200000
#define NUM_PTS_LARGE_ENOUGH_FOR_MAP 20000000


class NodeERASOR : public rclcpp::Node
{
public:
    // 结构体初始化
    NodeERASOR(const std::string & name);

    // 析构函数
    ~NodeERASOR()=default;

private:

    ERASOR_Config cfg;
    ERASOR erasor;

    rclcpp::TimerBase::SharedPtr timer_cre1;
    rclcpp::TimerBase::SharedPtr timer_cre2;

    PointCloudXYZI::Ptr query_voi_;
    PointCloudXYZI::Ptr map_voi_;
    PointCloudXYZI::Ptr map_outskirts_;
    PointCloudXYZI::Ptr map_arranged_, map_arranged_global_, map_arranged_complement_;

    /*** Outputs of ERASOR
     * map_filtered_ = map_static_estimate + map_egocentric_complement
     */
    PointCloudXYZI::Ptr map_static_estimate_;
    PointCloudXYZI::Ptr map_staticAdynamic;
    PointCloudXYZI::Ptr map_filtered_;
    PointCloudXYZI::Ptr map_egocentric_complement_;

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr node_cloud_dynamic_filter_publicher;

    PointCloudXYZI::Ptr raw_map;

    double submap_size_ = 200.0;
    double submap_center_x_, submap_center_y_;
    int num_pcs_init_;
    bool is_submap_not_initialized_ = true;

    double lidar_end_time;

    void declare_and_get_parameter();

    void setRawMap(PointCloudXYZI::Ptr const &raw_map);

    void VoxelPointCloud(const PointCloudXYZI::Ptr &cloud, PointCloudXYZI::Ptr &cloud_voxelized, const double voxel_size);

    const ERASOR_Config getCfg();

    void timer1_callback();

    void timer2_callback();

    void reassign_submap(double pose_x, double pose_y);

    // 设置子图
    void set_submap(const PointCloudXYZI &map_global, PointCloudXYZI &submap, PointCloudXYZI &submap_complement, double x, double y, double submap_size);

    // 它用于从给定的点云数据中提取感兴趣区域（VoI）。
    void fetch_VoI(double x_criterion, double y_criterion, PointCloudXYZI &query_pcd);

    //rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_;                     // 创建一个订阅器，用于订阅IMU数据。
    //rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_pcl_pc_;          // 创建一个订阅器，用于订阅原始点云数据。
    //rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr sub_pcl_livox_;   // 创建一个订阅器，用于订阅 Livox 点云数据。
    
};
