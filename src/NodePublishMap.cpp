#include"NodePublishMap.h"

NodePublishMap::NodePublishMap(const std::string & name):Node(name, rclcpp::NodeOptions().use_intra_process_comms(true))
{
    RCLCPP_INFO(this->get_logger(), "----NodePublishMap----");
    
    ground_pts.reset(new PointCloudXYZI);
    noise_cloud.reset(new PointCloudXYZI);
    raw_map_ptr_.reset(new PointCloudXYZI);

    Octomap_publisher = this->create_publisher<sensor_msgs::msg::PointCloud2>("/node_octomap", 10);

    count = 0;

    auto timer = std::chrono::milliseconds(static_cast<int64_t>(1000.0));
    timer_cre = rclcpp::create_timer(this, this->get_clock(), timer, std::bind(&NodePublishMap::publishMap, this));

    RCLCPP_INFO(this->get_logger(), "----NodePublishMap init finished.----"); // 输出信息，初始化结束
}

void NodePublishMap::publishMap(double timestamp) 
{
    PointCloudXYZI::Ptr octomap_map_(new PointCloudXYZI);

    VoxelPointCloud(raw_map_ptr_, raw_map_ptr_, config.resolution);
    std::cout << "raw_map_ptr_ size: " << raw_map_ptr_->size() << std::endl;
    //PointCloudXYZI::Ptr map_filtered(new PointCloudXYZI);
    //VoxelPointCloud(raw_map_ptr_, map_filtered, config.resolution);

    for (auto &pt : raw_map_ptr_->points)
    {
        octomap::point3d point(pt.x, pt.y, pt.z);

        octomap::OcTreeNode *node = m_octree->search(point);

        if (node == NULL) continue;

        if (m_octree->isNodeOccupied(node)) octomap_map_->push_back(pt);

    }

    VoxelPointCloud(octomap_map_, octomap_map_, config.resolution);

    if (octomap_map_->size() == 0)
    {
        RCLCPP_INFO(this->get_logger(), "octomap_map_ is empty, no map is saved");
        return;
    }

    sensor_msgs::msg::PointCloud2 laserCloudmsg;
    pcl::toROSMsg(*octomap_map_, laserCloudmsg);

    // laserCloudmsg.header.stamp = ros::Time().fromSec(lidar_end_time);
    laserCloudmsg.header.stamp = get_ros_time(timestamp);
    laserCloudmsg.header.frame_id = "camera_init";
    Octomap_publisher->publish(laserCloudmsg);

}


void NodePublishMap::setConfig()
{
    this->declare_parameter<float>("Octomap.resolution", 0.02);
    this->get_parameter_or<float>("Octomap.resolution", config.resolution, 0.02);
}