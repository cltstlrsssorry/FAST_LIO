#include <mutex>
#include <thread>
#include <fstream>
#include <csignal>
#include <chrono>
#include <unistd.h>

#include <octomap/octomap.h>
#include <octomap_msgs/msg/octomap.hpp>
#include <octomap_msgs/conversions.h>

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

class NodeOctomap : public rclcpp::Node
{
public:
    NodeOctomap(const std::string & name);

    // 析构函数
    ~NodeOctomap()=default;


private:

    rclcpp::TimerBase::SharedPtr timer_cre;

    rclcpp::Publisher<octomap_msgs::msg::Octomap>::SharedPtr publish_octomap;

    double timestamp;

    int count;

    Octomap_Config config;

    octomap::KeyRay m_keyRay;
    octomap::OcTreeKey m_updateBBXMin;
    octomap::OcTreeKey m_updateBBXMax;

    unsigned m_treeDepth;
    unsigned m_maxTreeDepth;

    PointCloudXYZI::Ptr ground_pts;
    PointCloudXYZI::Ptr raw_map_ptr_;

    PointCloudXYZI::Ptr noise_cloud;

    void setConfig();

    void timer_callback();

    void updateOccupancy();

    void publishOctomap();

    //void updateOccupancy(PointCloudXYZI::Ptr single_pc);

    void filterGroundPlane(PointCloudXYZI::Ptr const& pc, PointCloudXYZI::Ptr &ground, PointCloudXYZI::Ptr &nonground);


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
