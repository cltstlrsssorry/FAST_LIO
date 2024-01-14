#include <fastlio.h>

#include <rclcpp/rclcpp.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl/surface/mls.h>
#include <pcl/search/kdtree.h>
#include <pcl/surface/gp3.h>
#include <pcl/features/normal_3d.h>

#include <sensor_msgs/msg/point_cloud2.hpp>

class NodeReconstruction : public rclcpp::Node
{
public:
    NodeReconstruction(const std::string &name);

    // 析构函数
    ~NodeReconstruction() = default;

private:

    double timestamp;

    rclcpp::TimerBase::SharedPtr timer_cre;

    PointCloudXYZI::Ptr raw_map_ptr_;

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_smoothed;

    pcl::PointCloud<pcl::Normal>::Ptr cloud_normal;

    pcl::PolygonMesh triangles;

    void getConfig();

    void timer_callback();

    void CloudSmoothed(PointCloudXYZI::Ptr &cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr &smoothed);

    void CloudNormals(PointCloudXYZI::Ptr &cloud, pcl::PointCloud<pcl::Normal>::Ptr normals);

    void concatenateFields(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_xyz,
                           const pcl::PointCloud<pcl::Normal>::Ptr &cloud_normals,
                           const pcl::PointCloud<pcl::PointNormal>::Ptr &cloud_with_normals);

    void CloudTriangle(const pcl::PointCloud<pcl::PointNormal>::Ptr &cloud);
};
