#include "NodePublishMap.h"

NodePublishMap::NodePublishMap(const std::string &name) : Node(name, rclcpp::NodeOptions().use_intra_process_comms(true))
{
    RCLCPP_INFO(this->get_logger(), "----NodePublishMap----");

    publish_count = 1;

    filter_dynamic_map_ptr_.reset(new PointCloudXYZI());

    raw_map_ptr_.reset(new PointCloudXYZI());

    raw_map_voxel_ptr_.reset(new PointCloudXYZI());

    getConfig();

    Octomap_publisher = this->create_publisher<sensor_msgs::msg::PointCloud2>("/publish_down_dynamic_map", 10);

    auto timer = std::chrono::milliseconds(static_cast<int64_t>(1000.0 / 100.0));
    timer_cre = rclcpp::create_timer(this, this->get_clock(), timer, std::bind(&NodePublishMap::timer_callback, this));

    RCLCPP_INFO(this->get_logger(), "----NodePublishMap init finished.----"); // 输出信息，初始化结束
}

void NodePublishMap::timer_callback()
{
    if (raw_points_list.empty()) return;

    if (m_octree == NULL) return;

    timestamp = raw_points_list.begin()->time;
    PointCloudXYZI::Ptr temp_pc = raw_points_list.begin()->points;
    raw_points_list.pop_front();

    ++ publish_count;

    if (temp_pc->points.size() < 50) return;

    *raw_map_ptr_ += *temp_pc;

    if (publish_count % 2 == 0) VoxelPointCloud(raw_map_ptr_, raw_map_ptr_, uniform_size);

    if (publish_count % 10 == 0)
    {
        filter_dynamic_map.points->points.clear();

        for (auto &pt : raw_map_ptr_->points)
        {
            octomap::point3d point(pt.x, pt.y, pt.z);
            octomap::OcTreeNode *node = m_octree->search(point);

            if (node == NULL)
                continue;
            
            if (m_octree->isNodeOccupied(node)) filter_dynamic_map_ptr_->push_back(pt);
        }

        if (filter_dynamic_map_ptr_->size() == 0)
        {
            RCLCPP_INFO(this->get_logger(), "octomap_map_ is empty, no map is saved");
            return;
        }

        *raw_map_ptr_ += *filter_dynamic_map_ptr_;

        VoxelPointCloud(raw_map_ptr_, raw_map_ptr_, uniform_size);

        filter_dynamic_map.points = raw_map_ptr_;
        filter_dynamic_map.time = timestamp;

        sensor_msgs::msg::PointCloud2 laserCloudmsg;
        pcl::toROSMsg(*raw_map_ptr_, laserCloudmsg);

        // laserCloudmsg.header.stamp = ros::Time().fromSec(lidar_end_time);
        laserCloudmsg.header.stamp = get_ros_time(timestamp);
        laserCloudmsg.header.frame_id = "camera_init";
        Octomap_publisher->publish(laserCloudmsg);

        publish_count = 1;

    }

}

void NodePublishMap::getConfig()
{
    this->declare_parameter<double>("publishmap.size", 0.02);

    this->get_parameter_or<double>("publishmap.size", uniform_size, 0.02);
}

void NodePublishMap::VoxelPointCloud(PointCloudXYZI::Ptr &cloud, PointCloudXYZI::Ptr &cloud_voxelized, double size)
{
    if (cloud->empty())
        return;

    pcl::VoxelGrid<PointType> voxel_grid;
    voxel_grid.setInputCloud(cloud);
    voxel_grid.setLeafSize(size, size, size);
    voxel_grid.filter(*cloud_voxelized);
}

void NodePublishMap::UniformSamplingPointCloud(PointCloudXYZI::Ptr &cloud, PointCloudXYZI::Ptr &cloud_uniformed, double size)
{
    if (cloud->empty())
        return;

    pcl::UniformSampling<PointType> uniform_sampling;
    uniform_sampling.setInputCloud(cloud);
    uniform_sampling.setRadiusSearch(size);
    uniform_sampling.filter(*cloud_uniformed);
}