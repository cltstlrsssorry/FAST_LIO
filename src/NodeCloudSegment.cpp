#include "NodeCloudSegment.h"

NodeCloudSegment::NodeCloudSegment(const std::string & name,PointCloudXYZI::Ptr InputPoints):Node(name, rclcpp::NodeOptions().use_intra_process_comms(true))
{
    
    std::cout<<"cloud segment"<<std::endl;

    this->InputPoints=InputPoints;
    this->declare_parameter<bool>("publish.map_en", false);
    this->get_parameter_or<bool>("publish.map_en", map_pub_en, false);

    pubLaserCloudMap = this->create_publisher<sensor_msgs::msg::PointCloud2>("/Laser_map", 20);

    auto reg_map_period_ms = std::chrono::milliseconds(static_cast<int64_t>(5000.0));
    reg_map_pub_timer_ = rclcpp::create_timer(this, this->get_clock(), reg_map_period_ms, std::bind(&NodeCloudSegment::reg_map_publish_callback, this));

}

NodeCloudSegment::~NodeCloudSegment()
{

}

void NodeCloudSegment::reg_map_publish_callback()
{
    if (map_pub_en)
        publish_segment_map(pubLaserCloudMap);
}

//Laser_map
void NodeCloudSegment::publish_segment_map(rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publish_cloud)
{
    if (InputPoints->empty())
        return;
    // 均匀降采样率+

    PointCloudXYZI::Ptr points(new PointCloudXYZI());

    // *pcl_wait_pub 矩阵稀疏化
    pcl::VoxelGrid<PointType> voxel_grid;
    voxel_grid.setInputCloud(InputPoints);
    voxel_grid.setLeafSize(0.1, 0.1, 0.1);
    voxel_grid.filter(*InputPoints);

    // 点云分割
    pcl::search::Search<PointType>::Ptr tree(new pcl::search::KdTree<PointType>);
    tree->setInputCloud(InputPoints);
    std::vector<pcl::PointIndices> clusters;

    pcl::EuclideanClusterExtraction<PointType> ec;
    ec.setClusterTolerance(0.1); // 2cm
    ec.setMinClusterSize(50);
    ec.setMaxClusterSize(100000);
    ec.setSearchMethod(tree);
    ec.setInputCloud(InputPoints);
    ec.extract(clusters);

    for (std::vector<pcl::PointIndices>::const_iterator it = clusters.begin(); it != clusters.end(); it++)
    {
        PointCloudXYZI::Ptr cloud_cluster(new PointCloudXYZI());

        for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); pit++)
        {
            cloud_cluster->points.push_back(InputPoints->points[*pit]);
        }

        cloud_cluster->width = cloud_cluster->points.size();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;

        std::cout << cloud_cluster->size() << " ";

        *points += *cloud_cluster;
    }

    std::cout << endl;

    // 发布点云
    sensor_msgs::msg::PointCloud2 laserCloudmsg;
    pcl::toROSMsg(*points, laserCloudmsg);

    laserCloudmsg.header.stamp = fastlio::get_ros_time(lidar_end_time);
    laserCloudmsg.header.frame_id = "camera_init";
    publish_cloud->publish(laserCloudmsg);
}