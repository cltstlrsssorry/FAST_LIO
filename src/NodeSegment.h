#include <rclcpp/rclcpp.hpp>

#include <livox_ros_driver2/msg/custom_msg.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <pcl_conversions/pcl_conversions.h> //toROSMsg
#include <pcl/search/kdtree.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl/filters/uniform_sampling.h>
#include <pcl/filters/voxel_grid.h>

#include <pcl/segmentation/extract_clusters.h>

#include <fastlio.h>

using namespace std;

class NodeSegment : public rclcpp::Node
{
    
public:
    NodeSegment(const std::string &name);

    ~NodeSegment()=default;

private:
    void getConfig();

    void timer_callback();
    //Laser_map

    void UniformSamplingPointCloud(PointCloudXYZI::Ptr &cloud, PointCloudXYZI::Ptr &cloud_uniformed, double leaf_size);

    void VoxelPointCloud(PointCloudXYZI::Ptr &cloud, PointCloudXYZI::Ptr &cloud_voxelized, double leaf_size);

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudMap;

    rclcpp::TimerBase::SharedPtr reg_map_pub_timer_;

    Segment_Config config;

    PointCloudXYZI::Ptr filter_dynamic_map_ptr_;

    PointCloudXYZI::Ptr uniformed_map_ptr_;

    double timestamp;
    
    
};