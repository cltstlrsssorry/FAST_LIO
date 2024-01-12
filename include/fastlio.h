#ifndef FASTLIO_H
#define FASTLIO_H

#include <deque>
#include <rclcpp/rclcpp.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <sensor_msgs/msg/imu.hpp>

#include <octomap/octomap.h>

#define ANSI_RED "\033[1m\x1b[31m"
#define ANSI_GREEN "\033[1m\x1b[32m"
#define ANSI_YELLOW "\033[1m\x1b[33m"
#define ANSI_BLUE "\033[1m\x1b[34m"
#define ANSI_MAGENTA "\033[1m\x1b[35m"
#define ANSI_CYAN "\033[1m\x1b[36m"
#define ANSI_RESET "\x1b[0m"
#define ANSI_BOLD "\033[1m"

#define ROOT_DIR "@ ROOT_DIR @"

typedef pcl::PointXYZINormal PointType;

typedef pcl::PointCloud<PointType> PointCloudXYZI;

struct ERASOR_Config
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

struct Octomap_Config
{
    // main param, -1 means no range limit
    // Octomap.resolution：这是 Octomap 占用网格的分辨率，单位是米。
    // 这个值越小，生成的地图就越精细，但计算的复杂性也会增加。
    // 这里设置为 0.1，意味着每个网格的边长是 0.1 米。
    double resolution;

    // Octomap.maxRange 和 Octomap.minRange：这两个参数定义了传感器的最大和最小范围。
    // 这里都设置为 -1.0，可能表示这两个参数没有被使用，或者是使用默认值。
    float maxRange;
    float minRange;

    // Octomap.probHit 和 Octomap.probMiss：这两个参数是用于更新占用网格的概率模型的。
    // probHit 是当传感器检测到障碍物时，该网格被认为是占用的概率。
    // probMiss 是当传感器没有检测到障碍物时，该网格被认为是空闲的概率。
    float probHit;
    float probMiss;

    // Octomap.thresMin 和 Octomap.thresMax：这两个参数定义了一个网格被认为是占用或空闲的概率阈值。
    // 如果一个网格的占用概率高于 thresMax，那么它就被认为是占用的。
    // 如果一个网格的占用概率低于 thresMin，那么它就被认为是空闲的。
    float thresMin;
    float thresMax;

    float opccupancyThres;

    // hether prune tree
    bool m_prune;

    // ===> Ground Segmentation, so that we will not count ground
    bool filterGroundPlane;
    float m_groundFilterDistance;
    float m_groundFilterAngle;
    float m_groundFilterPlaneDistance;

    // ===> Filter
    bool filterNoise;
    int filterMeanK;
    float StddevMulThresh;

    bool verbose;
};

struct Segment_Config
{
    bool segment_en;
    bool publish_map_en;
    double uniform_size;
};

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