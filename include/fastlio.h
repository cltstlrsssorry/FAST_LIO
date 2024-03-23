#ifndef FASTLIO_H
#define FASTLIO_H

#include <deque>
#include <rclcpp/rclcpp.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <sensor_msgs/msg/imu.hpp>

#include <octomap/octomap.h>

#include <pcl/surface/gp3.h>

#define ANSI_RED "\033[1m\x1b[31m"
#define ANSI_GREEN "\033[1m\x1b[32m"
#define ANSI_YELLOW "\033[1m\x1b[33m"
#define ANSI_BLUE "\033[1m\x1b[34m"
#define ANSI_MAGENTA "\033[1m\x1b[35m"
#define ANSI_CYAN "\033[1m\x1b[36m"
#define ANSI_RESET "\x1b[0m"
#define ANSI_BOLD "\033[1m"

typedef pcl::PointXYZINormal PointType;

typedef pcl::PointCloud<PointType> PointCloudXYZI;

struct PointLists
{
    double time;

    PointCloudXYZI::Ptr points;

    PointLists();

    PointLists(double time, PointCloudXYZI::Ptr points);
};

extern PointCloudXYZI::Ptr featsFromMap;

// nodeoctomap
extern std::deque<PointLists> down_size_points_list;

// nodepublishmap
extern std::deque<PointLists> raw_points_list;

extern std::deque<PointLists> segment_map;

extern std::deque<PointLists> filter_dynamic_map;

extern std::deque<PointLists> filter_dynamic_map_PCA;

extern std::deque<PointLists> filter_dynamic_map_triangle;

extern std::deque<PointLists> wait_octomap_points_list;

extern std::deque<double> time_buffer;
extern std::deque<PointCloudXYZI::Ptr> lidar_buffer;
extern std::deque<sensor_msgs::msg::Imu::ConstSharedPtr> imu_buffer;

extern double last_timestamp_lidar, last_timestamp_imu;

double get_time_sec(const builtin_interfaces::msg::Time &time);

rclcpp::Time get_ros_time(double timestamp);

// for octomap
extern octomap::OcTree *m_octree;

#endif