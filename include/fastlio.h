#ifndef FASTLIO_H
#define FASTLIO_H

#include <deque>
#include <rclcpp/rclcpp.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <sensor_msgs/msg/imu.hpp>

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

struct Config
{
    /**< Parameters of MapUpdater*/
    double map_voxel_size_ = 0.2;
    double query_voxel_size_ = 0.2;
    int global_voxelization_period_;

    /**< Params. of Volume of Interest (VoI) */
    double max_range_;
    int num_rings_, num_sectors_;
    double min_h_, max_h_;
    double scan_ratio_threshold;

    double submap_size_;
    double submap_center_x_;
    double submap_center_y_;

    double th_seeds_heights_ = 0.5;
    double th_dist_ = 0.05;
    int num_lprs_ = 10;
    int minimum_num_pts = 6;
    int iter_groundfilter_ = 3;
    int num_lowest_pts = 5;
    bool verbose_ = true; // print out logs

    std::string mode = "naive";
    bool replace_intensity = false;
    int removal_interval_ = 2;

    // tf lidar to body
    double tf_x = 0.0;
    double tf_y = 0.0;
    double tf_z = 0.0;

    bool is_large_scale_ = false;
    
};

struct LocalPointLists
{
    double time;
    PointCloudXYZI::Ptr cloudpoint;

    LocalPointLists(double time, PointCloudXYZI::Ptr cloudpoint);

};

extern PointCloudXYZI::Ptr featsFromMap;
extern std::deque<LocalPointLists> featsFromMap_list;

extern std::deque<double> time_buffer;
extern std::deque<PointCloudXYZI::Ptr> lidar_buffer;
extern std::deque<sensor_msgs::msg::Imu::ConstSharedPtr> imu_buffer;

extern double last_timestamp_lidar, last_timestamp_imu ;

double get_time_sec(const builtin_interfaces::msg::Time &time);

rclcpp::Time get_ros_time(double timestamp);

#endif