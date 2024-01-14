#include <fastlio.h>

#include <rclcpp/rclcpp.hpp>

#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/octree/octree.h>
#include <pcl/octree/octree_pointcloud_changedetector.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/uniform_sampling.h>

struct CONFIG_CLOUDCOMPARE
{
    float resolution;

    float radius;

    float clusterTolerance;

    int minClusterSize;

    int maxClusterSize;
};

class NodeCloudCompare : public rclcpp::Node
{

public:
    NodeCloudCompare(const std::string &name);

    // 析构函数
    ~NodeCloudCompare() = default;

private:

    double distances[100000][720];

    bool inited;

    double timestamp;

    CONFIG_CLOUDCOMPARE config;

    PointCloudXYZI::Ptr last_cloud;

    PointCloudXYZI::Ptr curr_cloud;

    pcl::octree::OctreePointCloudChangeDetector<PointType>::Ptr octree_detect;

    rclcpp::TimerBase::SharedPtr timer_cre;

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr changed_points_pub;

    void setConfig();

    void timer_callback();

    void get_changed_points(PointCloudXYZI::Ptr &in_cloud, PointCloudXYZI::Ptr &out_cloud);

    void get_collection_changed_points(PointCloudXYZI::Ptr &in_cloud, PointCloudXYZI::Ptr &out_cloud);

    void getAngle();
  
};
