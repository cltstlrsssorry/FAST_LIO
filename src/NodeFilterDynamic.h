#include <mutex>
#include <thread>
#include <fstream>
#include <csignal>
#include <chrono>
#include <unistd.h>

#include <octomap/octomap.h>
#include <rclcpp/rclcpp.hpp>

#include <fastlio.h>

#include <pcl/point_cloud.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/common.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/uniform_sampling.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>

class NodeFilterDynamic : public rclcpp::Node
{
public:
        NodeFilterDynamic(const std::string &name);

        // 析构函数
        ~NodeFilterDynamic() = default;

private:
        int publish_count;
        double timestamp;

        double uniform_size;
        bool publish_map_en;

        rclcpp::TimerBase::SharedPtr timer_cre;

        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr Octomap_publisher;

        PointCloudXYZI::Ptr raw_map_ptr_;

        PointCloudXYZI::Ptr raw_map_ptr_seg_;

        void getConfig();

        void timer_callback();

        void VoxelPointCloud(PointCloudXYZI::Ptr &cloud, PointCloudXYZI::Ptr &cloud_voxelized, double voxel_size);

        void UniformSamplingPointCloud(PointCloudXYZI::Ptr &cloud, PointCloudXYZI::Ptr &cloud_uniformed, double leaf_size);

        void segmentcrave(PointCloudXYZI::Ptr &cloud, PointCloudXYZI::Ptr &cloud_segmented);

};
