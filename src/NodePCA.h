#include <fastlio.h>
#include <Eigen/Core>
#include <rclcpp/rclcpp.hpp>
#include <Eigen/Eigenvalues> // Include the necessary header file

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/centroid.h>
#include <pcl_conversions/pcl_conversions.h> //toROSMsg

#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/vector3.hpp>

#define PI 3.1415926

class NodePCA : public rclcpp::Node
{
public:
    NodePCA(const std::string & name);

    // 析构函数
    ~NodePCA()=default;

private:

    builtin_interfaces::msg::Time timestamp;

    int count,inited;

    bool publish_map_en;

    void setConfig();

    void timer_callback();

    PointCloudXYZI::Ptr pca_cloud_ptr_;

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_pca;

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLasercentroid;

    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pubLaserPCA_X;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pubLaserPCA_Y;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pubLaserPCA_Z;

    rclcpp::TimerBase::SharedPtr timer_cre;

    void VectorToQuaternionf(Eigen::Vector3f &pos, 
                                Eigen::Vector3f &vector,
                                rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pubLaserPCA);

    void cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);


};
