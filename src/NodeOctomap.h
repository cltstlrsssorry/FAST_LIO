#include <mutex>
#include <thread>
#include <fstream>
#include <csignal>
#include <chrono>
#include <unistd.h>

#include <rclcpp/rclcpp.hpp>

#include <octomap/octomap.h>
#include <octomap_msgs/msg/octomap.hpp>
#include <octomap_msgs/conversions.h>

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

struct CONFIG_OCTO
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

    CONFIG_OCTO config;

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
