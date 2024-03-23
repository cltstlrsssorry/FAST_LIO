#include <fastlio.h>

#include <rclcpp/rclcpp.hpp>

#include <pcl/io/ply_io.h>
#include <pcl/point_types.h> //pcl中支持的点类型头文件

#include <pcl/point_cloud.h>

#include <pcl/surface/mls.h>
#include <pcl/search/kdtree.h>
#include <pcl/surface/gp3.h>
#include <pcl/features/normal_3d.h>

#include <pcl_conversions/pcl_conversions.h>

#include <sensor_msgs/msg/point_cloud2.hpp>

class NodeReconstruction : public rclcpp::Node
{
public:
    NodeReconstruction(const std::string &name);

    // 析构函数
    ~NodeReconstruction() = default;

private:

    bool init;

    int count;
    
    builtin_interfaces::msg::Time timestamp;

    PointCloudXYZI::Ptr raw_map_reconstruction_ptr_;

    rclcpp::TimerBase::SharedPtr timer_reconstruction_;

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_reconstruction;
    
    void timer_callback();

    void cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

    void CloudSmoothed(PointCloudXYZI::Ptr &cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr &smoothed);

    void CloudNormals(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_smoothed, pcl::PointCloud<pcl::Normal>::Ptr normals);

    void ConcatenateFields(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_xyz,
                           const pcl::PointCloud<pcl::Normal>::Ptr &cloud_normals,
                           const pcl::PointCloud<pcl::PointNormal>::Ptr &cloud_with_normals);

    void CloudTriangle(const pcl::PointCloud<pcl::PointNormal>::Ptr &cloud);
    
};
