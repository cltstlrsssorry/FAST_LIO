#include <mutex>
#include <thread>
#include <fstream>
#include <csignal>
#include <chrono>
#include <unistd.h>

#include <octomap/octomap.h>
#include <octomap_msgs/msg/octomap.hpp>

#include <rclcpp/rclcpp.hpp>

#include<fastlio.h>

#include <pcl/point_cloud.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/common.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/segmentation/sac_segmentation.h>

class NodePublishMap : public rclcpp::Node
{
public:
    NodePublishMap(const std::string & name);

    // 析构函数
    ~NodePublishMap()=default;


private:

    Octomap_Config config;

    rclcpp::TimerBase::SharedPtr timer_cre;

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr Octomap_publisher;

    PointCloudXYZI::Ptr raw_map_ptr_;

    PointCloudXYZI::Ptr noise_cloud;

    void setConfig();

    void timer_callback();

    void publishMap(double timestamp);

    void createOctomap(PointCloudXYZI::Ptr &single_pc);


    void VoxelPointCloud(const PointCloudXYZI::Ptr& cloud, PointCloudXYZI::Ptr& cloud_voxelized, const double voxel_size);


protected:

    inline static void updateMinKey(const octomap::OcTreeKey& in, octomap::OcTreeKey& min) {
      for (unsigned i = 0; i < 3; ++i)
        min[i] = std::min(in[i], min[i]);
    };

    inline static void updateMaxKey(const octomap::OcTreeKey& in, octomap::OcTreeKey& max) {
      for (unsigned i = 0; i < 3; ++i)
        max[i] = std::max(in[i], max[i]);
    };


    
};
