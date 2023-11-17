#ifndef _FASTLIO_H_
#define _FASTLIO_H_

#include <deque>
#include <rclcpp/rclcpp.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <sensor_msgs/msg/imu.hpp>


typedef pcl::PointXYZINormal PointType;
typedef pcl::PointCloud<PointType> PointCloudXYZI;

extern std::deque<double> time_buffer;
extern std::deque<PointCloudXYZI::Ptr> lidar_buffer;
extern std::deque<sensor_msgs::msg::Imu::ConstSharedPtr> imu_buffer;

extern double last_timestamp_lidar, last_timestamp_imu ;

namespace fastlio{

    double get_time_sec(const builtin_interfaces::msg::Time &time);

    rclcpp::Time get_ros_time(double timestamp);
}

#endif