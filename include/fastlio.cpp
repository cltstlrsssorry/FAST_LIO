#include"fastlio.h"

PointCloudXYZI::Ptr featsFromMap(new PointCloudXYZI());

std::deque<PointLists> down_size_points_list;

std::deque<PointLists> raw_points_list;

std::deque<PointLists> segment_map;

std::deque<PointLists> filter_dynamic_map;

std::deque<PointLists> filter_dynamic_map_PCA;

octomap::OcTree* m_octree;

PointLists::PointLists()
{
    this->time = 0;
    this->points = PointCloudXYZI::Ptr(new PointCloudXYZI());
}

PointLists::PointLists(double time, PointCloudXYZI::Ptr points)
{
    this->time = time;
    this->points = points;
}

double get_time_sec(const builtin_interfaces::msg::Time &time)
{
    return rclcpp::Time(time).seconds();
}

rclcpp::Time get_ros_time(double timestamp)
{
    int32_t sec = std::floor(timestamp);
    auto nanosec_d = (timestamp - std::floor(timestamp)) * 1e9;
    uint32_t nanosec = nanosec_d;
    return rclcpp::Time(sec, nanosec);
}

