#include <Eigen/Core>
#include <rclcpp/rclcpp.hpp>

#include <livox_ros_driver2/msg/custom_msg.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h> //toROSMsg
#include <pcl/search/kdtree.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/segmentation/extract_clusters.h>

#include <fastlio.h>

using namespace std;

class NodeCloudSegment : public rclcpp::Node
{
    
public:
    NodeCloudSegment(const std::string & name,PointCloudXYZI::Ptr InputPoints);

    ~NodeCloudSegment();

private:

    void reg_map_publish_callback();
    //Laser_map
    void publish_segment_map(rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publish_cloud);

    rclcpp::Time Get_time(double timestamp);

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudMap;
    rclcpp::TimerBase::SharedPtr reg_map_pub_timer_;

    PointCloudXYZI::Ptr InputPoints;

    bool map_pub_en = false;
    double lidar_end_time;
    
};